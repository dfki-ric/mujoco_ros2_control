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

namespace mujoco_ros2_control {
    MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node) : nh_(node) {
        // set up the parameter listener
        param_listener_ = std::make_shared<ParamListener>(nh_);
        param_listener_->refresh_dynamic_parameters();

        params_ = param_listener_->get_params();

        // Check that ROS has been initialized
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(nh_->get_logger(), "Unable to initialize Mujoco node.");
            return;
        }

        // create publisher for the Clock
        publisher_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SystemDefaultsQoS());
        clock_publisher_ = std::make_unique<ClockPublisher>(publisher_);

        // mujoco related parameters
        show_gui_ = params_.show_gui;
        real_time_factor_ = params_.real_time_factor;
        pub_clock_frequency_ = params_.clock_publisher_frequency;

        init_mujoco();

        init_controller_manager();

        // Start MuJoCo
        mj_resetData(mujoco_model_, mujoco_data_);

        // compute forward kinematics for new pos
        mj_forward(mujoco_model_, mujoco_data_);

        // run simulation to setup the new pos
        mj_step(mujoco_model_, mujoco_data_);
        mujoco_start_time_ = mujoco_data_->time;

        clock_gettime(CLOCK_MONOTONIC, &startTime_);
#ifdef USE_LIBSIMULATE
        mj_vis_.init(mujoco_model_, mujoco_data_);
#else
        mj_vis_.init(mujoco_model_, mujoco_data_, show_gui_);
#endif

        registerSensors();
        RCLCPP_INFO(nh_->get_logger(), "Sim environment setup complete");
    }

    MujocoRos2Control::~MujocoRos2Control()
    {
        stop_.store(true);
        for (auto &thread : camera_threads_) {
            thread.join();
        }
        executor_->remove_node(controller_manager_);
        executor_->cancel();
        thread_executor_spin_.join();
        // deallocate existing mjModel
        mj_deleteModel(mujoco_model_);

        // deallocate existing mjData
        mj_deleteData(mujoco_data_);

        mj_vis_.terminate();
    }

    void MujocoRos2Control::update() {
        mjtNum simstart = mujoco_data_->time;
        timespec currentTime{};
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        // run until the next frame must be rendered with 60Hz
        if (mj_vis_.sim->run) {
            while( mujoco_data_->time - simstart < 1.0/60.0 ) {
                // check that mujoco is not faster than the expected realtime factor
                clock_gettime(CLOCK_MONOTONIC, &currentTime);
                if (double (currentTime.tv_sec-startTime_.tv_sec) + double (currentTime.tv_nsec-startTime_.tv_nsec) / 1e9 >= (mujoco_data_->time-mujoco_start_time_)*params_.real_time_factor) {
                    publish_sim_time();
                    rclcpp::Time sim_time_ros = rclcpp::Time((int64_t) (mujoco_data_->time * 1e+9), RCL_ROS_TIME);
                    rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

                    // check if we should update the controllers
                    if (sim_period >= control_period_) {
                        // store simulation time
                        last_update_sim_time_ros_ = sim_time_ros;
                        // update the robot simulation with the state of the mujoco model
                        controller_manager_->read(sim_time_ros, sim_period);
                        // compute the controller commands
                        controller_manager_->update(sim_time_ros, sim_period);
                        // update the mujoco model with the result of the controller
                        controller_manager_->write(sim_time_ros, sim_period);
                    }
                    // Calculate the next mujoco step
                    mj_step(mujoco_model_, mujoco_data_);
                }
            }
        } else {
            mj_forward(mujoco_model_, mujoco_data_);
            mj_vis_.sim->speed_changed = true;
        }
        mj_vis_.update();
    }

    void MujocoRos2Control::publish_sim_time() {
        double sim_time = mujoco_data_->time;
        if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_) < 1.0/pub_clock_frequency_)
            return;
        if (clock_publisher_->trylock()) {
            clock_publisher_->msg_.clock.sec = std::floor(sim_time);
            clock_publisher_->msg_.clock.nanosec = std::floor((sim_time-std::floor(sim_time))*1e9);
            clock_publisher_->unlockAndPublish();
            last_pub_clock_time_ = sim_time;
        }
    }

    void MujocoRos2Control::init_mujoco() {
        char error[1000];

        // create mjModel
        mujoco_model_ = mj_loadXML(params_.robot_model_path.c_str(), NULL, error, 1000);

        if (!mujoco_model_) {
            RCLCPP_FATAL(nh_->get_logger(), "Could not load mujoco model with error: %s.\n", error);
            return;
        } else {
            // No problem with margins
            RCLCPP_INFO(nh_->get_logger(), "loaded mujoco model");
        }

        // Set simulation frequency
        mujoco_model_->opt.timestep = 1.0 / params_.simulation_frequency;

        // create mjData corresponding to mjModel
        mujoco_data_ = mj_makeData(mujoco_model_);
        if (!mujoco_data_)
        {
            RCLCPP_FATAL(nh_->get_logger(), "Could not create mujoco data from model.");
            return;
        }else {
            RCLCPP_INFO(nh_->get_logger(), "Created mujoco data");
        }

        // get the Mujoco simulation period as ros duration
        mujoco_period_ = rclcpp::Duration::from_seconds(mujoco_model_->opt.timestep);
    }

    void MujocoRos2Control::init_controller_manager() {
        RCLCPP_INFO(nh_->get_logger(), "init controller manager");

        resource_manager_ = std::make_unique<mujoco_ros2_control::MujocoResourceManager>(nh_, mujoco_model_, mujoco_data_);

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
            RCLCPP_ERROR(nh_->get_logger(), "The controller period (%f) is faster than the simulation period (%f).",
                         control_period_.seconds(), mujoco_period_.seconds());
            control_period_ = mujoco_period_;
        } else if (control_period_ > mujoco_period_) {
            if (control_period_ < mujoco_period_) {
                RCLCPP_WARN(nh_->get_logger(), "The controller period (%f) is slower than the simulation period (%f).",
                            control_period_.seconds(), mujoco_period_.seconds());
            }
        }

        // Force setting of use_sime_time parameter
        controller_manager_->set_parameter(
        rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

        stop_ = false;
        auto spin = [this]() {
            while(!stop_.load()) {
                executor_->spin_once();
            }
        };
        thread_executor_spin_ = std::thread(spin);
    }


    void MujocoRos2Control::registerSensors() {
        // Add sensors
        if (mujoco_model_->nsensor > 0) {
            std::map<std::string, mujoco_ros2_sensors::MujocoRos2Sensors::Sensors> sensors;
            for (int id = 0; id < mujoco_model_->nsensor; id++) {
                mujoco_ros2_sensors::MujocoRos2Sensors::Sensors sensor;
                int obj_id = mujoco_model_->sensor_objid[id];
                int obj_type = mujoco_model_->sensor_objtype[id];
                std::string obj_name = mj_id2name(mujoco_model_, obj_type, obj_id);
                int sensor_type = mujoco_model_->sensor_type[id];
                std::string sensor_name = mj_id2name(mujoco_model_, mjOBJ_SENSOR, id);
                int sensor_adr = mujoco_model_->sensor_adr[id];
                int sensor_dim = mujoco_model_->sensor_dim[id];
                if (sensors.find(obj_name) == sensors.end()) {
                    sensors.insert(std::make_pair(obj_name, sensor));
                    sensors.at(obj_name).obj_type = obj_type;
                }
                sensors.at(obj_name).sensor_ids.push_back(id);
                sensors.at(obj_name).sensor_types.push_back(sensor_type);
                sensors.at(obj_name).sensor_names.push_back(sensor_name);
                sensors.at(obj_name).sensor_addresses.push_back(sensor_adr);
                sensors.at(obj_name).sensor_dimensions.push_back(sensor_dim);
            }
            mujoco_ros2_sensors_ = std::make_shared<mujoco_ros2_sensors::MujocoRos2Sensors>(executor_, mujoco_model_, mujoco_data_, sensors);
        }

        // Add cameras
        if (mujoco_model_->ncam > 0) {
            cameras_.resize(mujoco_model_->ncam);
            for (int id = 0; id < mujoco_model_->ncam; id++) {
                std::string name = mj_id2name(mujoco_model_, mjOBJ_CAMERA, id);
                auto node = camera_nodes_.emplace_back(rclcpp::Node::make_shared(name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
                executor_->add_node(node);
                cameras_.at(id).reset(new mujoco_rgbd_camera::MujocoDepthCamera(node, mujoco_model_, mujoco_data_, id,
                                                                                name, &stop_));
                camera_threads_.emplace_back([ObjectPtr = cameras_.at(id)] {ObjectPtr->update();});
            }
        }
    }

}  // namespace mujoco_ros_control


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

    // create an executor and spin the created node with it
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([ObjectPtr = &executor] { ObjectPtr->spin(); });

    while ( rclcpp::ok())
    {
        mujoco_ros2_control_plugin.update();
    }
    executor_thread.join();

    return 0;
}