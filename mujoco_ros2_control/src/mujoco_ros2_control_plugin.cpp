// Copyright (c) 2013, Open Source Robotics Foundation. All rights reserved.
// Copyright (c) 2013, The Johns Hopkins University. All rights reserved.
// Modifications copyright (C) 2013, DFKI GmbH, Robotics Innovation Center. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Dave Coleman, Jonathan Bohren, Adrian Danzglock
   Desc:   Mujoco plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

namespace mujoco_ros2_control
{
    MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node) : model_node_(node)
    {
        model_node_->declare_parameter<std::string>("robot_description_param", "robot_description");
        model_node_->declare_parameter<std::string>("robot_description_node", "robot_state_publisher");
        model_node_->declare_parameter<std::string>("robot_model_path", "");

        model_node_->declare_parameter("robot_joints", std::vector<std::string>{""});
        model_node_->declare_parameter<std::string>("params_file_path", "");
        model_node_->declare_parameter<bool>("visualization", true);

        // Check that ROS has been initialized
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(model_node_->get_logger(), "Unable to initialize Mujoco node.");
            return;
        }

        // publish clock for simulated time
        //pub_clock_ = model_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        publisher_ = model_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SystemDefaultsQoS());
        clock_publisher_ = std::make_unique<ClockPublisher>(publisher_);

        // read urdf from ros parameter server then setup actuators and mechanism control node.
        robot_description_param_ = model_node_->get_parameter("robot_description_param").as_string();
        robot_description_node_ = model_node_->get_parameter("robot_description_node").as_string();
        robot_model_path_ = model_node_->get_parameter("robot_model_path").as_string();
        std::string params_file_path = model_node_->get_parameter("params_file_path").as_string();
        mujoco_vis_ = model_node_->get_parameter("visualization").as_bool();

        auto rcl_context = model_node_->get_node_base_interface()->get_context()->get_rcl_context();
        std::vector<std::string> arguments = {"--ros-args"};

        arguments.emplace_back(RCL_PARAM_FILE_FLAG);
        arguments.push_back(params_file_path);

        std::vector<const char *> argv;
        argv.reserve(arguments.size());
        for (const auto & arg : arguments) {
            argv.push_back(reinterpret_cast<const char *>(arg.data()));
        }
        rcl_arguments_t rcl_args = rcl_get_zero_initialized_arguments();
        rcl_ret_t rcl_ret = rcl_parse_arguments(
                static_cast<int>(argv.size()),
                argv.data(), rcl_get_default_allocator(), &rcl_args);
        rcl_context->global_arguments = rcl_args;
        if (rcl_ret != RCL_RET_OK) {
            RCLCPP_ERROR(model_node_->get_logger(), "parser error %s\n", rcl_get_error_string().str);
            rcl_reset_error();
            return;
        }
        if (rcl_arguments_get_param_files_count(&rcl_args) < 1) {
            RCLCPP_ERROR(
                    model_node_->get_logger(), "failed to parse input yaml file(s)");
            return;
        }

        std::string urdf_string;
        std::vector<hardware_interface::HardwareInfo> control_hardware_info;
        try {
            urdf_string = get_urdf(robot_description_param_);
            control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
        } catch (const std::runtime_error & ex) {
            RCLCPP_ERROR_STREAM(
                    model_node_->get_logger(),
                    "Error parsing URDF in mujoco_ros2_control plugin, plugin not active : " << ex.what());
            rclcpp::shutdown();
        }
        char error[1000];

        // create mjModel
        mujoco_model_ = mj_loadXML(robot_model_path_.c_str(), NULL, error, 1000);
        if (!mujoco_model_)
        {
            RCLCPP_ERROR(model_node_->get_logger(), "Could not load mujoco model with error: %s.\n", error);
            return;
        } else {
            RCLCPP_INFO(model_node_->get_logger(), "loaded mujoco model");
        }

        mujoco_model_->opt.timestep = 0.001;

        // create mjData corresponding to mjModel
        mujoco_data_ = mj_makeData(mujoco_model_);
        if (!mujoco_data_)
        {
            printf("Could not create mujoco data from model.\n");
            return;
        }else {
            RCLCPP_INFO(model_node_->get_logger(), "Created mujoco data");
        }

        // get the Mujoco simulation period
        mujoco_period_ = rclcpp::Duration::from_seconds(mujoco_model_->opt.timestep);

        // set control period as mujoco_period_
        control_period_ = mujoco_period_;

        // load the RobotHWSim abstraction to interface the controllers with the mujoco model
        resource_manager_ = std::make_unique<hardware_interface::ResourceManager>();
        try
        {
            robot_hw_sim_loader_.reset(
                    new pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>
                            ("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
            robot_hw_sim_loader_->createUnmanagedInstance("mujoco_ros2_control/MujocoSystem");
            urdf::Model urdf_model;
            const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

            for (auto & i : control_hardware_info) {
                std::string robot_hw_sim_type_str_ = i.hardware_class_type;
                auto mujocoSystem = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(
                        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
                if (!mujocoSystem->initSim(
                        mujoco_model_,
                        mujoco_data_,
                        i,
                        urdf_model_ptr))
                {
                    RCLCPP_FATAL(
                            model_node_->get_logger(), "Could not initialize robot simulation interface");
                    return;
                }

                resource_manager_->import_component(std::move(mujocoSystem), i);

                // activate all components
                rclcpp_lifecycle::State state(
                        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                        hardware_interface::lifecycle_state_names::ACTIVE);
                resource_manager_->set_component_state(i.name, state);
            }
        }
        catch(pluginlib::LibraryLoadException &ex)
        {
            RCLCPP_FATAL_STREAM(model_node_->get_logger() , "Failed to create robot sim interface loader: "
                    << ex.what());
        }
        RCLCPP_INFO(model_node_->get_logger(), "Loaded mujoco_ros_control.");
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        // Create the controller manager
        RCLCPP_INFO(model_node_->get_logger(), "Loading controller_manager");
        controller_manager_.reset(
                new controller_manager::ControllerManager(
                        std::move(resource_manager_),
                        executor_,
                        "controller_manager",
                        model_node_->get_namespace()));
        executor_->add_node(controller_manager_);

        if (!controller_manager_->has_parameter("update_rate")) {
            RCLCPP_ERROR_STREAM(
                    model_node_->get_logger(), "controller manager doesn't have an update_rate parameter");
            return;
        }

        int cm_update_rate = controller_manager_->get_parameter("update_rate").as_int();
        control_period_ = rclcpp::Duration(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));
        // Check the period against the simulation period
        if (control_period_ < mujoco_period_) {
            RCLCPP_ERROR_STREAM(
                    model_node_->get_logger(),
                    "Desired controller update period (" << control_period_.seconds() <<
                                                         " s) is faster than the mujoco simulation period (" <<
                                                         mujoco_period_.seconds() << " s).");
        } else if (control_period_ > mujoco_period_) {
            RCLCPP_WARN_STREAM(
                    model_node_->get_logger(),
                    " Desired controller update period (" << control_period_.seconds() <<
                                                          " s) is slower than the mujoco simulation period (" <<
                                                          mujoco_period_.seconds() << " s).");
        }

        // Force setting of use_sime_time parameter
        controller_manager_->set_parameter(
                rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

        stop_ = false;
        auto spin = [this]()
        {
            while (rclcpp::ok() && !stop_) {
                executor_->spin_once();
            }
        };
        thread_executor_spin_ = std::thread(spin);


        mj_resetData(mujoco_model_, mujoco_data_);

        // compute forward kinematics for new pos
        mj_forward(mujoco_model_, mujoco_data_);

        // run simulation to setup the new pos
        mj_step(mujoco_model_, mujoco_data_);
        mujoco_start_time_ = mujoco_data_->time;
        //system_start_time_ = std::chrono::system_clock::now();
        clock_gettime(CLOCK_MONOTONIC, &startTime_);
        RCLCPP_INFO(model_node_->get_logger(), "Sim environment setup complete");
        if (mujoco_vis_) {
            mj_vis_.init(mujoco_model_, mujoco_data_);
        }
    }

    MujocoRos2Control::~MujocoRos2Control() 
    {
        stop_ = true;
        executor_->remove_node(controller_manager_);
        executor_->cancel();
        thread_executor_spin_.join();
        // deallocate existing mjModel
        mj_deleteModel(mujoco_model_);

        // deallocate existing mjData
        mj_deleteData(mujoco_data_);
        mj_deactivate();
    }

    // from https://github.com/shadow-robot/mujoco_ros_pkgs/blob/kinetic-devel/mujoco_ros_control/src/mujoco_ros_control.cpp
    void MujocoRos2Control::update() {
        mjtNum simstart = mujoco_data_->time;
        while( mujoco_data_->time - simstart < 1.0/60.0 ) {
            clock_gettime(CLOCK_MONOTONIC, &currentTime_);
            double duration_since_start = (currentTime_.tv_sec-startTime_.tv_sec) + (currentTime_.tv_nsec-startTime_.tv_nsec) / 1e9;
            if (duration_since_start >= mujoco_data_->time-mujoco_start_time_) {
                publish_sim_time();
                rclcpp::Time sim_time_ros = rclcpp::Time((int64_t) (mujoco_data_->time * 1e+9), RCL_ROS_TIME);

                rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

                //mj_step1(mujoco_model_, mujoco_data_);

                // check if we should update the controllers
                if (sim_period >= control_period_) {
                    // store simulation time
                    last_update_sim_time_ros_ = sim_time_ros;
                    // compute the controller commands
                    controller_manager_->update(sim_time_ros, sim_period);

                    // update the robot simulation with the state of the mujoco model
                    controller_manager_->read(sim_time_ros, sim_period);

                    // update the mujoco model with the result of the controller
                    controller_manager_->write(sim_time_ros, sim_period);
                }
                mj_step(mujoco_model_, mujoco_data_);
            }
        }
        if (mujoco_vis_) {
            mj_vis_.update();
        }
    }

    // get the URDF XML from the parameter server
    std::string MujocoRos2Control::get_urdf(const std::string& param_name) const
    {
        std::string urdf_string;

        using namespace std::chrono_literals;
        std::shared_ptr<rclcpp::SyncParametersClient> parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
                this->parameter_node_, robot_description_node_);
        while (!parameters_client->wait_for_service(0.5s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                        parameter_node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                        robot_description_node_.c_str());
                return "";
            }
            RCLCPP_ERROR(
                    parameter_node_->get_logger(), "%s service not available, waiting again...",
                    robot_description_node_.c_str());
        }

        RCLCPP_INFO(
                parameter_node_->get_logger(), "connected to service %s.", robot_description_node_.c_str());

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            RCLCPP_INFO(parameter_node_->get_logger(), "param_name %s", param_name.c_str());

            try {
                auto parameters = parameters_client->get_parameters({param_name});
                urdf_string = parameters.at(0).value_to_string();
            } catch (const std::exception & e) {
                RCLCPP_ERROR(parameter_node_->get_logger(), "%s", e.what());
            }

            if (!urdf_string.empty()) {
                break;
            } else {
                RCLCPP_WARN(
                        parameter_node_->get_logger(), "mujoco_ros2_control plugin is waiting for model"
                                                 " URDF in parameter [%s] on the ROS param server.", param_name.c_str());
            }
            usleep(100000);
        }
        RCLCPP_INFO(
                parameter_node_->get_logger(), "Recieved urdf from param server, parsing...");

        return urdf_string;
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
        //rosgraph_msgs::msg::Clock ros_time;
        //ros_time.clock.set__sec(std::floor(sim_time));
        //ros_time.clock.set__nanosec(std::floor((sim_time-ros_time.clock.sec)*1e9));
        // publish time to ros
        //pub_clock_->publish(ros_time);
        //last_pub_clock_time_ = sim_time;
    }
}  // namespace mujoco_ros_control



// it also works without gui
int main(int argc, char** argv)
{
    // partially from https://github.com/shadow-robot/mujoco_ros_pkgs/blob/kinetic-devel/mujoco_ros_control/src/mujoco_ros_control.cpp
    // (the gui related parts and the while loop are from there)

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mujoco_node");

    mujoco_ros2_control::MujocoRos2Control mujoco_ros2_control_plugin(node);

    // spin
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