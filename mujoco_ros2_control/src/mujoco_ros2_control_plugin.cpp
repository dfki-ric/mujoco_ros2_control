/**
* @file mujoco_ros2_control_plugin.cpp
*
* @brief This file contains the implementation of the Mujoco ROS2 Control plugin.
*
* @author Adrian Danzglock
* @date 2023
* @license BSD 3-Clause License
* @copyright Copyright (c) 2023, DFKI GmbH
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions
*    and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
*    and the following disclaimer in the documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
* The init_controller_manager method contains a modified version of the original code from Cyberbotics Ltd.
* https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_control/src/Ros2Control.cpp
*
* Original code licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
        *
        *     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
        * limitations under the License.
*/

#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

#include <sys/stat.h>

#include <optional>

#include "ament_index_cpp/get_package_prefix.hpp"

// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;
constexpr auto kPausedClockHeartbeatPeriod = std::chrono::milliseconds(100);

namespace mujoco_ros2_control {
MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node) : nh_(node) {
  // set up the parameter listener
  param_listener_ = std::make_shared<ParamListener>(nh_);
  param_listener_->refresh_dynamic_parameters();

  params_ = param_listener_->get_params();

  // Check that ROS has been initialized
  if (!rclcpp::ok()) {
    RCLCPP_FATAL(nh_->get_logger(), "Unable to initialize Mujoco node.");
    return;
  }

  // create publisher for the Clock
  publisher_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SystemDefaultsQoS());
  clock_publisher_ = std::make_unique<ClockPublisher>(publisher_);

  reset_notify_publisher_ = nh_->create_publisher<std_msgs::msg::Empty>(
      "/mujoco_reset_notify", rclcpp::SystemDefaultsQoS());

  // create services for play/pause and reset
  mujoco_play_pause_service_ = nh_->create_service<std_srvs::srv::Trigger>(
      "mujoco_play_pause",
      std::bind(&MujocoRos2Control::mujocoPlayPauseCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  mujoco_reset_service_ = nh_->create_service<std_srvs::srv::Trigger>(
      "mujoco_reset",
      std::bind(&MujocoRos2Control::mujocoResetCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  mujoco_step_simulation_service_ = nh_->create_service<mujoco_ros2_control::srv::StepSimulation>(
      "mujoco_step_simulation",
      std::bind(&MujocoRos2Control::mujocoStepSimulationCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // mujoco related parameters
  show_gui_ = params_.show_gui;
  real_time_factor_ = params_.real_time_factor;
  pub_clock_frequency_ = params_.clock_publisher_frequency;
  running_.store(!params_.synchronous_mode, std::memory_order_release);

  // Everything below dereferences mujoco_model_/mujoco_data_, so a model that
  // failed to load has to end the setup here. main() reports the failure and
  // exits; the destructor is safe on this half-built object.
  if (!init_mujoco()) {
    return;
  }

  if (mujoco_model_->nkey > 0) {
    mj_resetDataKeyframe(mujoco_model_, mujoco_data_, 0);
  } else {
    mj_resetData(mujoco_model_, mujoco_data_);
  }

  // compute forward kinematics for new pos
  mj_forward(mujoco_model_, mujoco_data_);

  // run simulation to setup the new pos
  mj_step(mujoco_model_, mujoco_data_);
  mujoco_start_time_ = mujoco_data_->time;

  // Initialize MuJoCo completely before starting the controller-manager
  // executor. Hardware initialization calls mj_forward() while registering
  // joints; starting that executor earlier allowed it to race the mj_step()
  // above and corrupt MuJoCo's constraint workspace.
  init_controller_manager();

  clock_gettime(CLOCK_MONOTONIC, &startTime_);

  registerSensors();

  if (show_gui_) {
    mjdata_to_render_ = *mujoco_data_;
    mj_vis_.init(mujoco_model_, &mjdata_to_render_);
    mj_vis_.setResetFlag(&reset_requested_);
  }

  thread_sim_ = std::thread(&MujocoRos2Control::update, this);
  initialized_ = true;
  RCLCPP_INFO(nh_->get_logger(), "Sim environment setup complete");
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_.store(true);
  for (auto &thread : camera_threads_) {
    thread.join();
  }
  for (auto &thread : lidar_threads_) {
    thread.join();
  }
  // Joins the sensor threads and stops spinning their nodes. Has to complete
  // before mj_deleteModel/mj_deleteData below: those threads read both.
  ros2_plugins_.shutdown();
  if (thread_sim_.joinable()) {
    thread_sim_.join();
  }
  if (thread_executor_spin_.joinable()) {
    thread_executor_spin_.join();
  }
  cameras_.clear();
  // deallocate existing mjModel
  mj_deleteModel(mujoco_model_);

  // deallocate existing mjData
  mj_deleteData(mujoco_data_);

  if (show_gui_ && initialized_) {
    mj_vis_.terminate();
  }

}

void MujocoRos2Control::render() {
  if (!show_gui_) return;
  mj_vis_.update();
}

void MujocoRos2Control::update() {
  bool initial_clock_heartbeat_sent = false;
  while (!stop_.load()) {
    // Keep ROS time available even while controller-manager hardware is still
    // configuring. This is especially important in synchronous mode because
    // no autonomous physics step exists to publish the first clock sample.
    if (params_.synchronous_mode) {
      const auto wall_now = std::chrono::steady_clock::now();
      bool sent_initial_heartbeat = false;
      if (wall_now - last_clock_wall_publish_time_ >=
          kPausedClockHeartbeatPeriod) {
        std::lock_guard<std::mutex> step_lock(step_mutex_);
        std::lock_guard<std::mutex> sim_lock(sim_mutex_);
        publish_sim_time(true);
        last_clock_wall_publish_time_ = wall_now;
        sent_initial_heartbeat = !initial_clock_heartbeat_sent;
        initial_clock_heartbeat_sent = true;
      }
      if (sent_initial_heartbeat) {
        // Give the controller-manager executor one short scheduling window to
        // consume its first /clock sample before its first update cycle.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }

    if (!system_configured_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // Handle a GUI reset even while the simulation is paused.
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      resetSimulation();
    }

    if (params_.synchronous_mode) {
      // Controller switches and action-goal handoff must still be serviced
      // while physics time is frozen. This is deliberately a zero-duration
      // controller cycle: only StepSimulation is allowed to call mj_step().
      {
        std::lock_guard<std::mutex> step_lock(step_mutex_);
        std::lock_guard<std::mutex> sim_lock(sim_mutex_);
        // controller_manager rejects a zero timestamp when use_sim_time is
        // enabled. MuJoCo legitimately returns to t=0 after reset, so use the
        // smallest non-zero controller timestamp for this zero-duration
        // housekeeping cycle. Physics time and /clock remain exactly at zero.
        const int64_t simulation_nanoseconds =
          static_cast<int64_t>(mujoco_data_->time * 1e9);
        const rclcpp::Time sim_time(
          std::max<int64_t>(simulation_nanoseconds, 1), RCL_ROS_TIME);
        const rclcpp::Duration zero_period(0, 0);
        controller_manager_->read(sim_time, zero_period);
        controller_manager_->update(sim_time, zero_period);
        controller_manager_->write(sim_time, zero_period);

      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const bool keep_running = show_gui_ ? mj_vis_.sim->run : running_.load();
    if (!keep_running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    timespec currentTime{};
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();

    // check that mujoco is not faster than the expected realtime factor
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    if (double(currentTime.tv_sec - startTime_.tv_sec) +
        double(currentTime.tv_nsec - startTime_.tv_nsec) / 1e9 >=
      (mujoco_data_->time - mujoco_start_time_) * params_.real_time_factor) {
      std::lock_guard<std::mutex> step_lock(step_mutex_);
      advanceSimulationStep();

      if (show_gui_) {
        mujoco::MutexLock lock(mj_vis_.sim->mtx);
        mjdata_to_render_ = *mujoco_data_;
      }
    }
  }
}

void MujocoRos2Control::updateControllersAtCurrentTime() {
  const rclcpp::Time sim_time_ros(
    static_cast<int64_t>(mujoco_data_->time * 1e9), RCL_ROS_TIME);
  const rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;
  if (sim_period < control_period_) {
    return;
  }
  last_update_sim_time_ros_ = sim_time_ros;
  controller_manager_->read(sim_time_ros, sim_period);
  controller_manager_->update(sim_time_ros, sim_period);
  controller_manager_->write(sim_time_ros, sim_period);
}

void MujocoRos2Control::advanceSimulationStep() {
  {
    std::lock_guard<std::mutex> sim_lock(sim_mutex_);
    updateControllersAtCurrentTime();
    // The <mujoco_ros2_plugin> plugins that asked to run per physics step. Both
    // stepping paths -- the autonomous loop and the StepSimulation service --
    // come through here, and both hold step_mutex_ around it, so these hooks see
    // the simulation exclusively and need no locking of their own. Inputs go in
    // before the step, observations after it.
    ros2_plugins_.beforeStep(mujoco_data_);
    mj_step(mujoco_model_, mujoco_data_);
    ros2_plugins_.afterStep(
      mujoco_data_,
      rclcpp::Time(static_cast<int64_t>(mujoco_data_->time * 1e9), RCL_ROS_TIME));
  }
  publish_sim_time();
}

void MujocoRos2Control::resetSimulation() {
  std::lock_guard<std::mutex> step_lock(step_mutex_);
  {
    std::lock_guard<std::mutex> sim_lock(sim_mutex_);
    if (mujoco_model_->nkey > 0) {
      mj_resetDataKeyframe(mujoco_model_, mujoco_data_, 0);
    } else {
      mj_resetData(mujoco_model_, mujoco_data_);
    }
    mj_forward(mujoco_model_, mujoco_data_);
    mujoco_start_time_ = mujoco_data_->time;
    last_update_sim_time_ros_ = rclcpp::Time(
      static_cast<int64_t>(mujoco_data_->time * 1e9), RCL_ROS_TIME);
    last_pub_clock_time_ = -1.0;
  }
  clock_gettime(CLOCK_MONOTONIC, &startTime_);
  publish_sim_time();
  reset_notify_publisher_->publish(std_msgs::msg::Empty());
}

void MujocoRos2Control::publish_sim_time(bool force) {
  double sim_time = mujoco_data_->time;
  if (!force && pub_clock_frequency_ > 0 &&
      (sim_time - last_pub_clock_time_) < 1.0 / pub_clock_frequency_)
    return;
  if (clock_publisher_->trylock()) {
    const int64_t simulation_nanoseconds =
      static_cast<int64_t>(std::floor(sim_time * 1e9));
    // rclcpp::Clock::started() deliberately reports false for ROS time zero.
    // A synchronous reset legitimately leaves MuJoCo at t=0, but repeatedly
    // publishing that value makes controller_manager warn that no /clock was
    // received during its paused housekeeping cycles. Publish the smallest
    // representable initialized ROS timestamp until the first physics step;
    // service responses, sensor stamps, and MuJoCo time remain exactly zero.
    const int64_t published_nanoseconds = params_.synchronous_mode
      ? std::max<int64_t>(simulation_nanoseconds, 1)
      : simulation_nanoseconds;
    clock_publisher_->msg_.clock.sec =
      static_cast<int32_t>(published_nanoseconds / 1000000000LL);
    clock_publisher_->msg_.clock.nanosec =
      static_cast<uint32_t>(published_nanoseconds % 1000000000LL);
    clock_publisher_->unlockAndPublish();
    last_pub_clock_time_ = sim_time;
  }
}

namespace {

// mj_loadAllPluginLibraries() takes a plain C callback with no user-data
// argument, so the logger it reports through has to live at file scope. The scan
// runs once, from the constructor, before any other thread exists.
std::optional<rclcpp::Logger> plugin_scan_logger;

// Reports what a single library contributed to MuJoCo's global plugin table.
// `first` is the slot of its first plugin, or -1 when it registered none.
void on_plugin_library_loaded(const char *filename, int first, int count) {
  if (!plugin_scan_logger) return;

  if (count <= 0) {
    // Every shared library in the directory is dlopen'ed, so hitting one that
    // registers nothing is expected rather than an error.
    RCLCPP_DEBUG(*plugin_scan_logger, "  %s: registered no plugins", filename);
    return;
  }

  for (int slot = first; slot < first + count; slot++) {
    const mjpPlugin *plugin = mjp_getPluginAtSlot(slot);
    RCLCPP_INFO(*plugin_scan_logger, "  %s: registered '%s'", filename,
                (plugin && plugin->name) ? plugin->name : "<unnamed>");
  }
}

bool is_regular_file(const std::string &path) {
  struct stat info{};
  return stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

bool is_directory(const std::string &path) {
  struct stat info{};
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

}  // namespace

bool MujocoRos2Control::load_mujoco_plugins() {
  // Unlike simulate's main(), libmujoco never scans for plugin libraries itself,
  // so anything referenced from an <extension> block has to be dlopen'ed before
  // the model is compiled.
  std::vector<std::string> directories = params_.mujoco_plugin_directories;
  const bool using_default_directory = directories.empty();

  if (using_default_directory) {
    try {
      directories.push_back(
        ament_index_cpp::get_package_prefix("mujoco_ros2_control") + "/lib/mujoco_plugin");
    } catch (const ament_index_cpp::PackageNotFoundError &e) {
      RCLCPP_WARN(nh_->get_logger(),
        "Could not resolve the mujoco_ros2_control prefix, skipping the default plugin "
        "directory: %s", e.what());
    }
  }

  const int plugins_before = mjp_pluginCount();
  plugin_scan_logger = nh_->get_logger();

  for (const auto &directory : directories) {
    if (!is_directory(directory)) {
      // A missing default directory just means no plugins were installed, while
      // a configured one that is missing is almost certainly a typo.
      if (using_default_directory) {
        RCLCPP_DEBUG(nh_->get_logger(),
          "No MuJoCo plugin directory at '%s'.", directory.c_str());
      } else {
        RCLCPP_WARN(nh_->get_logger(),
          "MuJoCo plugin directory '%s' does not exist, skipping it.", directory.c_str());
      }
      continue;
    }
    RCLCPP_INFO(nh_->get_logger(), "Scanning '%s' for MuJoCo plugins", directory.c_str());
    mj_loadAllPluginLibraries(directory.c_str(), &on_plugin_library_loaded);
  }

  for (const auto &library : params_.mujoco_plugin_libraries) {
    if (!is_regular_file(library)) {
      // mj_loadPluginLibrary() reports a failed dlopen through mju_error, which
      // terminates the process, so bad paths are caught here instead.
      RCLCPP_FATAL(nh_->get_logger(),
        "MuJoCo plugin library '%s' does not exist.", library.c_str());
      plugin_scan_logger.reset();
      return false;
    }
    const int before_library = mjp_pluginCount();
    mj_loadPluginLibrary(library.c_str());
    on_plugin_library_loaded(
      library.c_str(), before_library, mjp_pluginCount() - before_library);
  }

  plugin_scan_logger.reset();

  const int registered = mjp_pluginCount() - plugins_before;
  if (registered > 0) {
    RCLCPP_INFO(nh_->get_logger(), "Registered %d MuJoCo plugin(s)", registered);
  }
  return true;
}

bool MujocoRos2Control::init_mujoco() {
  char error[1000];

  // Plugins have to be registered before the compiler resolves <extension>.
  if (!load_mujoco_plugins()) {
    return false;
  }

  // create mjModel
  mujoco_model_ = mj_loadXML(params_.robot_model_path.c_str(), NULL, error, 1000);

  if (!mujoco_model_) {
    RCLCPP_FATAL(nh_->get_logger(), "Could not load mujoco model with error: %s.\n", error);
    return false;
  } else {
    // No problem with margins
    RCLCPP_INFO(nh_->get_logger(), "loaded mujoco model");
  }

  // Set simulation frequency
  mujoco_model_->opt.timestep = 1.0 / params_.simulation_frequency;

  // create mjData corresponding to mjModel
  mujoco_data_ = mj_makeData(mujoco_model_);
  if (!mujoco_data_) {
    RCLCPP_FATAL(nh_->get_logger(), "Could not create mujoco data from model.");
    return false;
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Created mujoco data");
  }

  // get the Mujoco simulation period as ros duration
  mujoco_period_ = rclcpp::Duration::from_seconds(mujoco_model_->opt.timestep);
  return true;
}

    void MujocoRos2Control::init_controller_manager() {
        RCLCPP_INFO(nh_->get_logger(), "init controller manager");

        resource_manager_ = std::make_unique<mujoco_ros2_control::MujocoResourceManager>(nh_, mujoco_model_, mujoco_data_, &system_configured_);

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();
        cm_node_options.parameter_overrides({{"use_sim_time", true}});
        controller_manager_.reset(
            new controller_manager::ControllerManager(
            std::move(resource_manager_),
            executor_, 
            "controller_manager",
            "",
            cm_node_options
        ));
        
        executor_->add_node(controller_manager_);
        
        if (!controller_manager_->has_parameter("update_rate")) {
            RCLCPP_ERROR(nh_->get_logger(), "controller manager doesn't have an update_rate parameter");
            return;
        }

  long cm_update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));
  // Check the period against the simulation period
  if (control_period_ < mujoco_period_) {
    RCLCPP_ERROR(nh_->get_logger(),
      "The controller period (%f) is faster than the simulation period (%f).",
      control_period_.seconds(), mujoco_period_.seconds());
    control_period_ = mujoco_period_;
  } else if (control_period_ > mujoco_period_) {
    if (control_period_ < mujoco_period_) {
      RCLCPP_WARN(nh_->get_logger(),
        "The controller period (%f) is slower than the simulation period (%f).",
        control_period_.seconds(), mujoco_period_.seconds());
    }
  }

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_ = false;
  auto spin = [this]() {
    // read CPU affinity
    rclcpp::Parameter cpu_affinity_param;
    if (controller_manager_->get_parameter("cpu_affinity", cpu_affinity_param)) {
      std::vector<int> cpus = {};
      if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        cpus = {static_cast<int>(cpu_affinity_param.as_int())};
      } else if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto cpu_affinity_param_array = cpu_affinity_param.as_integer_array();
        std::for_each(
          cpu_affinity_param_array.begin(), cpu_affinity_param_array.end(),
          [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
      }
      const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
      if (!affinity_result.first) {
        RCLCPP_WARN(
          controller_manager_->get_logger(), "Unable to set the CPU affinity : '%s'",
          affinity_result.second.c_str());
      }
    }

    // read thread priority
    const int thread_priority =
      controller_manager_->get_parameter_or<int>("thread_priority", kSchedPriority);
    RCLCPP_INFO(
      controller_manager_->get_logger(), "Spawning %s RT thread with scheduler priority: %d",
      controller_manager_->get_name(), thread_priority);

    if (!realtime_tools::configure_sched_fifo(thread_priority)) {
      RCLCPP_WARN(controller_manager_->get_logger(),
        "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
        "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
        "for details on how to enable realtime scheduling.",
        errno, strerror(errno));
    } else {
      RCLCPP_INFO(controller_manager_->get_logger(),
        "Successful set up FIFO RT scheduling policy with priority %i.", thread_priority);
    }

    // execute the executor of the controller_manager_
    while (rclcpp::ok() && !stop_.load()) {
      executor_->spin_once();
    }
  };
  thread_executor_spin_ = std::thread(spin);
}

void MujocoRos2Control::registerSensors() {
  // Parsed up front, ahead of the prefix-based discovery below: a MuJoCo site
  // claimed by a <mujoco_ros2_plugin> is skipped there, so declaring a camera or
  // a lidar on a site that also matches a discovery prefix creates one instance
  // rather than two.
  const auto ros2_plugin_declarations =
    MujocoRos2PluginLoader::parse(params_.robot_description, nh_->get_logger());
  std::set<std::string> claimed_sites;
  for (const auto &declaration : ros2_plugin_declarations) {
    // The name is claimed too: a declaration naming no site falls back to it.
    claimed_sites.insert(declaration.name);
    for (const char *key : {"site", "optical_site", "depth_site"}) {
      const auto parameter = declaration.parameters.find(key);
      if (parameter != declaration.parameters.end() && !parameter->second.empty()) {
        claimed_sites.insert(parameter->second);
      }
    }
  }

  // Add cameras declared as MuJoCo <camera> elements. These are always created;
  // their pose and vertical FOV come from the model camera (mjCAMERA_FIXED).
  for (int id = 0; id < mujoco_model_->ncam; id++) {
    std::string name = mj_id2name(mujoco_model_, mjOBJ_CAMERA, id);
    auto node = camera_nodes_.emplace_back(rclcpp::Node::make_shared(
      name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
    executor_->add_node(node);
    // A <camera> serves both color and depth from one viewpoint, so the optical
    // and depth mount ids are the same camera id (single render pass).
    auto camera = std::make_shared<mujoco_rgbd_camera::MujocoDepthCamera>(
      node, mujoco_model_, mujoco_data_, &sim_mutex_, name, &stop_,
      mujoco_rgbd_camera::MujocoDepthCamera::Mount::FixedCamera, id, id);
    cameras_.push_back(camera);
    camera_threads_.emplace_back([ObjectPtr = camera] { ObjectPtr->update(); });
  }

  // Add site-based cameras. Sites are classified by three prefixes and grouped by
  // the stripped name into one camera node:
  //   mjCam_<name>       -> one site for both color and depth (single render pass)
  //   mjCamOpt_<name>    -> color/optical site of a two-frame camera
  //   mjCamDepth_<name>  -> depth site of a two-frame camera (rendered separately)
  // A camera is created only if its node's params set enabled=true.
  {
    auto probe_node = rclcpp::Node::make_shared(
      "_mujoco_rgbd_camera_probe",
      rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    auto probe_listener = std::make_shared<mujoco_rgbd_camera::ParamListener>(probe_node);
    const auto cam_params = probe_listener->get_params();
    const std::string both_prefix = cam_params.site_prefix;
    const std::string opt_prefix = cam_params.optical_site_prefix;
    const std::string depth_prefix = cam_params.depth_site_prefix;

    // node name -> (optical site id, depth site id), -1 = absent.
    struct CamSites { int optical_id = -1; int depth_id = -1; };
    std::map<std::string, CamSites> cam_sites;

    for (int id = 0; id < mujoco_model_->nsite; id++) {
      const char *site_name_c = mj_id2name(mujoco_model_, mjOBJ_SITE, id);
      if (!site_name_c) continue;
      const std::string site_name(site_name_c);
      if (claimed_sites.count(site_name)) continue;

      // Match the most specific prefix first (mjCamOpt_/mjCamDepth_ also start with
      // "mjCam" but not with "mjCam_", so they never collide with both_prefix).
      auto matches = [&](const std::string &p) {
        return !p.empty() && site_name.rfind(p, 0) == 0;
      };
      if (matches(opt_prefix)) {
        std::string name = site_name.substr(opt_prefix.size());
        if (!name.empty()) cam_sites[name].optical_id = id;
      } else if (matches(depth_prefix)) {
        std::string name = site_name.substr(depth_prefix.size());
        if (!name.empty()) cam_sites[name].depth_id = id;
      } else if (matches(both_prefix)) {
        std::string name = site_name.substr(both_prefix.size());
        if (!name.empty()) { cam_sites[name].optical_id = id; cam_sites[name].depth_id = id; }
      }
    }

    for (const auto &[node_name, sites] : cam_sites) {
      auto node = rclcpp::Node::make_shared(
        node_name,
        rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));

      const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
      const auto it = overrides.find("enabled");
      if (it == overrides.end() || !it->second.get<bool>()) {
        continue;
      }

      camera_nodes_.push_back(node);
      executor_->add_node(node);
      auto camera = std::make_shared<mujoco_rgbd_camera::MujocoDepthCamera>(
        node, mujoco_model_, mujoco_data_, &sim_mutex_, node_name, &stop_,
        mujoco_rgbd_camera::MujocoDepthCamera::Mount::Site, sites.optical_id, sites.depth_id);
      cameras_.push_back(camera);
      camera_threads_.emplace_back([ObjectPtr = camera] { ObjectPtr->update(); });
    }
  }

 {
    auto probe_node = rclcpp::Node::make_shared(
      "_mujoco_gl_lidar_probe",
      rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    auto probe_listener = std::make_shared<mujoco_gl_lidar::ParamListener>(probe_node);
    const std::string prefix = probe_listener->get_params().site_prefix;

    for (int id = 0; id < mujoco_model_->nsite; id++) {
      const char *site_name_c = mj_id2name(mujoco_model_, mjOBJ_SITE, id);
      if (!site_name_c) continue;
      std::string site_name(site_name_c);
      if (claimed_sites.count(site_name)) continue;
      if (prefix.empty() || site_name.rfind(prefix, 0) != 0) continue;

      // The node (and therefore its params block and topics) is named without the
      // discovery prefix, e.g. site "lidar_head" -> node "head" -> "/head/...".
      std::string node_name = site_name.substr(prefix.size());
      if (node_name.empty()) node_name = site_name;

      auto node = rclcpp::Node::make_shared(
        node_name,
        rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));

      const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
      const auto it = overrides.find("enabled");
      if (it == overrides.end() || !it->second.get<bool>()) {
        continue;
      }

      lidar_nodes_.push_back(node);
      executor_->add_node(node);
      auto lidar = std::make_shared<mujoco_gl_lidar::MujocoGLLidar>(
        node, mujoco_model_, mujoco_data_, &sim_mutex_, id, node_name, &stop_);
      lidars_.push_back(lidar);
      lidar_threads_.emplace_back([ObjectPtr = lidar] { ObjectPtr->update(); });
    }
  }

  // Sensors declared as <mujoco_ros2_plugin> in the robot description. These are
  // read straight out of the raw URDF: they sit outside every <ros2_control>
  // block, so ros2_control's parser never reports them and no hardware
  // component owns them.
  ros2_plugins_.registerPlugins(
    ros2_plugin_declarations, mujoco_model_, mujoco_data_, &sim_mutex_, &step_mutex_,
    &stop_, nh_->get_logger());
}

void mujoco_ros2_control::MujocoRos2Control::mujocoPlayPauseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  bool now_running;
  if (show_gui_) {
    mj_vis_.sim->run = !mj_vis_.sim->run;
    now_running = mj_vis_.sim->run;
  } else {
    // Headless: there's no sim->run field, so use the dedicated atomic.
    now_running = !running_.load(std::memory_order_acquire);
    running_.store(now_running, std::memory_order_release);
  }
  response->success = true;
  response->message = now_running ? "Simulation running." : "Simulation paused.";
}

void mujoco_ros2_control::MujocoRos2Control::mujocoResetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  resetSimulation();
  response->success = true;
  response->message = "Simulation reset completed.";
}


void mujoco_ros2_control::MujocoRos2Control::mujocoStepSimulationCallback(
    const std::shared_ptr<mujoco_ros2_control::srv::StepSimulation::Request> request,
    std::shared_ptr<mujoco_ros2_control::srv::StepSimulation::Response> response) {
  if (!params_.synchronous_mode) {
    response->success = false;
    response->message = "Synchronous stepping is disabled.";
    return;
  }
  if (request->steps == 0 || request->steps > static_cast<uint32_t>(params_.max_step_batch)) {
    response->success = false;
    response->message = "steps must be in [1, " + std::to_string(params_.max_step_batch) + "].";
    return;
  }

  std::lock_guard<std::mutex> step_lock(step_mutex_);
  for (uint32_t i = 0; i < request->steps; ++i) {
    advanceSimulationStep();
  }
  // Publish the post-step joint state before returning to the Gym client.
  {
    std::lock_guard<std::mutex> sim_lock(sim_mutex_);
    updateControllersAtCurrentTime();
  }
  publish_sim_time();

  // A periodic RGB-D thread may have captured an intermediate state while the
  // batch was advancing. Force one render after the final step and wait until
  // it has been published before returning the authoritative simulation time.
  // The outer step lock freezes reset/teleport/physics throughout this wait.
  // Cameras reach the simulation two ways -- found by site prefix into cameras_,
  // or declared as a <mujoco_ros2_plugin> and owned by the plugin -- and both
  // have to be waited for. Everything is asked before anything is waited on, so
  // the frames are rendered concurrently.
  std::vector<std::pair<
      std::shared_ptr<mujoco_rgbd_camera::MujocoDepthCamera>, std::uint64_t>>
      camera_requests;
  camera_requests.reserve(cameras_.size());
  for (const auto &camera : cameras_) {
    camera_requests.emplace_back(camera, camera->request_synchronous_frame());
  }
  const auto plugin_requests = ros2_plugins_.requestSynchronousFrames();

  for (const auto &[camera, sequence] : camera_requests) {
    if (!camera->wait_for_synchronous_frame(
        sequence, std::chrono::seconds(5))) {
      response->success = false;
      response->message = "Timed out waiting for post-step RGB-D frame.";
      return;
    }
  }
  std::string stalled_plugin;
  if (!ros2_plugins_.waitForSynchronousFrames(
      plugin_requests, std::chrono::seconds(5), &stalled_plugin)) {
    response->success = false;
    response->message =
      "Timed out waiting for a post-step frame from plugin '" + stalled_plugin + "'.";
    return;
  }

  const int64_t nanoseconds = static_cast<int64_t>(mujoco_data_->time * 1e9);
  response->simulation_time.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  response->simulation_time.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  response->success = true;
  response->message = "Advanced " + std::to_string(request->steps) + " physics steps.";
}

}  // namespace mujoco_ros2_control

/**
 * @brief Main function for the Mujoco ROS2 Control plugin.
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return Exit code of the program.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mujoco_ros2_control");
  // create the mujoco_ros2_control_plugin
  mujoco_ros2_control::MujocoRos2Control mujoco_ros2_control_plugin(node);

  if (!mujoco_ros2_control_plugin.initialized()) {
    RCLCPP_FATAL(node->get_logger(),
      "Mujoco simulation setup failed; shutting down.");
    rclcpp::shutdown();
    return 1;
  }

  // create an executor and spin the created node with it
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([ObjectPtr = &executor] { ObjectPtr->spin(); });

  // The render loop only has work to do when the simulate GUI is up.
  if (mujoco_ros2_control_plugin.gui_enabled()) {
    while (rclcpp::ok()) {
      mujoco_ros2_control_plugin.render();
    }
  } else {
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  executor_thread.join();

  return 0;
}
