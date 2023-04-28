/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   mujoco_ros_control.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
**/


//#include <boost/bind.hpp>
#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"
#include "mujoco_ros2_control/mujoco_system.hpp"
#include <urdf/urdf/model.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

namespace mujoco_ros2_control
{
    MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node) : model_node_(node)
    {
        n_free_joints_ = 0;
        model_node_->declare_parameter<std::string>("robot_description_param", "robot_description");
        model_node_->declare_parameter<std::string>("robot_description_node", "robot_state_publisher");
        model_node_->declare_parameter<std::string>("robot_model_path", "/home/ubuntu22/ros2_ws/src/kuka_lbr_ros/kuka_lbr_mujoco/config/kuka_lbr.urdf");
        model_node_->declare_parameter("robot_joints", std::vector<std::string>{""});
        model_node_->declare_parameter<std::string>("params_file_path", "/home/ubuntu22/ros2_ws/src/kuka_lbr_ros/kuka_lbr_mujoco/config/ros2_controllers.yaml");

        // Check that ROS has been initialized
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(model_node_->get_logger(), "Unable to initialize Mujoco node.");
            return;
        }

        // publish clock for simulated time
        pub_clock_ = model_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // create robot node handle
        //robot_node_handle = ros::NodeHandle("/");

        //ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control node in namespace: %s", robot_namespace_.c_str());

        // read urdf from ros parameter server then setup actuators and mechanism control node.
        robot_description_param_ = model_node_->get_parameter("robot_description_param").as_string();
        robot_description_node_ = model_node_->get_parameter("robot_description_node").as_string();
        robot_model_path_ = model_node_->get_parameter("robot_model_path").as_string();
        std::string params_file_path = model_node_->get_parameter("params_file_path").as_string();

        auto rcl_context = model_node_->get_node_base_interface()->get_context()->get_rcl_context();
        std::vector<std::string> arguments = {"--ros-args"};

        arguments.push_back(RCL_PARAM_FILE_FLAG);
        arguments.push_back(params_file_path);

        std::vector<const char *> argv;
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

        // create mjData corresponding to mjModel
        mujoco_data_ = mj_makeData(mujoco_model_);
        if (!mujoco_data_)
        {
            printf("Could not create mujoco data from model.\n");
            return;
        }else {
            RCLCPP_INFO(model_node_->get_logger(), "Created mujoco data");
        }

        // check number of dofs
        get_number_of_dofs();

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
            /**robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHWSim");
            urdf::Model urdf_model;
            const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

            // get robot links from urdf
            std::map<std::string, boost::shared_ptr<urdf::Link> > robot_links;
            robot_links = urdf_model_ptr->links_;
            urdf_model.get
            std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator it;
            for (it = robot_links.begin(); it != robot_links.end(); ++it)
            {
                robot_link_names_.push_back(it->first);
            }*/
            urdf::Model urdf_model;
            const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

            // check for objects
            check_objects_in_scene();


            for (auto & i : control_hardware_info) {
                std::string robot_hw_sim_type_str_ = i.hardware_class_type;
                auto mujocoSystem = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(
                        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));

                rclcpp::Node::SharedPtr node_ros2 = std::dynamic_pointer_cast<rclcpp::Node>(this->model_node_);
                if (!mujocoSystem->initSim(
                        node_ros2,
                        mujoco_model_,
                        mujoco_data_,
                        i,
                        urdf_model_ptr,
                        n_free_joints_))
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


        // set up the initial simulation environment
        // TODO: Check if required, when commented in there is a problem with number of constraints
        //setup_sim_environment(control_hardware_info);
    }

    MujocoRos2Control::~MujocoRos2Control() 
    {
        // deallocate existing mjModel
        mj_deleteModel(mujoco_model_);

        // deallocate existing mjData
        mj_deleteData(mujoco_data_);
        mj_deactivate();
    }

    void MujocoRos2Control::setup_sim_environment(const std::vector<hardware_interface::HardwareInfo>& hwinfo)
    {
        size_t i = 0;
        for(const auto& hw : hwinfo) {
            for (const auto& joint : hw.joints) {
                for (const auto& interface : joint.state_interfaces) {
                    if (interface.name == "position") {
                        mujoco_data_->qpos[i] = std::stod(interface.initial_value);
                        i++;
                    }
                }
            }
        }

        //std::vector<std::string> robot_joints = model_node_->get_parameter("robot_joints").as_string_array();
        //for (size_t i = 0; i < robot_joints.size(); i++) {
        //    std::string param_name = "robot_initial_state." + robot_joints.at(i);
        //    model_node_->declare_parameter<double>(param_name, 0.0);
        //    mujoco_data_->qpos[i] = model_node_->get_parameter(param_name).as_double();
        //}

        // compute forward kinematics for new pos
        mj_forward(mujoco_model_, mujoco_data_);

        // run simulation to setup the new pos
        mj_step(mujoco_model_, mujoco_data_);

        RCLCPP_INFO(model_node_->get_logger(), "Sim environment setup complete");
    }

    void MujocoRos2Control::update()
    {
        publish_sim_time();

        rclcpp::Time sim_time = (rclcpp::Time)(mujoco_data_->time * 1e+9);
        rclcpp::Time sim_time_ros = (rclcpp::Time)(sim_time.nanoseconds());

        rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

        mj_step1(mujoco_model_, mujoco_data_);

        // check if we should update the controllers
        if (sim_period >= control_period_)
        {
            // store simulation time
            last_update_sim_time_ros_ = sim_time_ros;

            // update the robot simulation with the state of the mujoco model
            robot_hw_sim_->read(sim_time_ros, sim_period);

            // compute the controller commands
            controller_manager_->update(sim_time_ros, sim_period);
        }

        // update the mujoco model with the result of the controller
        robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);

        last_write_sim_time_ros_ = sim_time_ros;
        mj_step2(mujoco_model_, mujoco_data_);

        publish_objects_in_scene();
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
                //std::stringstream ss;
                // Get a few of the parameters just set.
                //for (auto & parameter : parameters)
                //{
                //    ss << "\nParameter name: " << parameter.get_name();
                //    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
                //       parameter.value_to_string();
                //    std::cout << ss.str() << std::endl;
                //}
            } catch (const std::exception & e) {
                RCLCPP_ERROR(parameter_node_->get_logger(), "%s", e.what());
            }

            if (!urdf_string.empty()) {
                break;
            } else {
                RCLCPP_ERROR(
                        parameter_node_->get_logger(), "mujoco_ros2_control plugin is waiting for model"
                                                 " URDF in parameter [%s] on the ROS param server.", param_name.c_str());
            }
            usleep(100000);
        }
        RCLCPP_INFO(
                parameter_node_->get_logger(), "Recieved urdf from param server, parsing...");

        return urdf_string;
    }

    std::string MujocoRos2Control::geom_type_to_string(int geom_type)
    {
        std::string result;
        switch (geom_type)
        {
            case 0 :
                result = mujoco_ros2_msgs::msg::ModelStates::PLANE;
                break;
            case 1 :
                result = mujoco_ros2_msgs::msg::ModelStates::HFIELD;
                break;
            case 2 :
                result = mujoco_ros2_msgs::msg::ModelStates::SPHERE;
                break;
            case 3 :
                result = mujoco_ros2_msgs::msg::ModelStates::CAPSULE;
                break;
            case 4 :
                result = mujoco_ros2_msgs::msg::ModelStates::ELLIPSOID;
                break;
            case 5 :
                result = mujoco_ros2_msgs::msg::ModelStates::CYLINDER;
                break;
            case 6 :
                result = mujoco_ros2_msgs::msg::ModelStates::BOX;
                break;
            case 7 :
                result = mujoco_ros2_msgs::msg::ModelStates::MESH;
                break;
            default:
                result = "unknown_type";
                break;
        }
        return result;
    }

    void MujocoRos2Control::get_number_of_dofs()
    {
        n_dof_ = mujoco_model_->njnt;
    }

    void MujocoRos2Control::publish_sim_time()
    {
        rclcpp::Time sim_time = (rclcpp::Time) (mujoco_data_->time * 1e+9);
        if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).seconds() < 1.0/pub_clock_frequency_)
            return;
        rclcpp::Time current_time = (rclcpp::Time) (mujoco_data_->time * 1e+9);
        rosgraph_msgs::msg::Clock ros_time_;
        //ros_time_.clock.fromSec(current_time.toSec());
        ros_time_.clock.set__nanosec(current_time.nanoseconds());
        ros_time_.clock.set__sec(current_time.seconds());
        // publish time to ros
        last_pub_clock_time_ = sim_time;
        pub_clock_->publish(ros_time_);
    }

    void MujocoRos2Control::check_objects_in_scene()
    {
        int num_of_bodies = mujoco_model_->nbody;
        int object_id;
        int joint_addr;
        int joint_type;
        int num_of_joints_for_body;
        std::string object_name;

        for (object_id=0; object_id < num_of_bodies; object_id++)
        {
            object_name = mj_id2name(mujoco_model_, 1, object_id);
            num_of_joints_for_body = mujoco_model_->body_jntnum[object_id];
            if (0 == num_of_joints_for_body &&
                !(std::find(robot_link_names_.begin(), robot_link_names_.end(), object_name) != robot_link_names_.end()))
            {
                objects_in_scene_[object_id] = STATIC;
                RCLCPP_INFO(model_node_->get_logger(), "Static object found: %s", object_name.c_str());
            }
            else if (1 == num_of_joints_for_body)
            {
                joint_addr = mujoco_model_->body_jntadr[object_id];
                joint_type = mujoco_model_->jnt_type[joint_addr];
                if (0 == joint_type)
                {
                    objects_in_scene_[object_id] = FREE;
                    n_free_joints_++;
                    RCLCPP_INFO(model_node_->get_logger(), "Free object found: %s", object_name.c_str());
                }
            }
        }
    }

    void MujocoRos2Control::publish_objects_in_scene()
    {
        const int geom_size_dim = 3;
        const int xpos_dim = 3;
        const int xquat_dim = 4;
        int geom_type;
        int geom_addr;
        geometry_msgs::msg::Pose pose;
        std_msgs::msg::Float64MultiArray size;
        mujoco_ros2_msgs::msg::ModelStates objects;

        for (auto & it : objects_in_scene_)
        {
            size.data.clear();
            geom_addr = mujoco_model_->body_geomadr[it.first];
            geom_type = mujoco_model_->geom_type[geom_addr];

            for (int i=0; i < geom_size_dim; i++)
            {
                size.data.push_back(mujoco_model_->geom_size[3 * geom_addr + i]);
            }

            pose.position.x = mujoco_data_->xpos[xpos_dim * it.first];
            pose.position.y = mujoco_data_->xpos[xpos_dim * it.first + 1];
            pose.position.z = mujoco_data_->xpos[xpos_dim * it.first + 2];
            pose.orientation.x = mujoco_data_->xquat[xquat_dim * it.first + 1];
            pose.orientation.y = mujoco_data_->xquat[xquat_dim * it.first + 2];
            pose.orientation.z = mujoco_data_->xquat[xquat_dim * it.first + 3];
            pose.orientation.w = mujoco_data_->xquat[xquat_dim * it.first];

            objects.name.emplace_back(mj_id2name(mujoco_model_, 1, it.first));
            objects.type.push_back(geom_type_to_string(geom_type));
            objects.is_static.push_back(it.second);
            objects.size.push_back(size);
            objects.pose.push_back(pose);
        }

        objects_in_scene_publisher->publish(objects);
    }
}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::cout << " " << std::endl;

    //ros::NodeHandle nh_;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("model_node");

    mujoco_ros2_control::MujocoRos2Control mujoco_ros2_control_plugin(node);

    mujoco_ros2_control::MujocoVisualizationUtils &mujoco_visualization_utils =
            mujoco_ros2_control::MujocoVisualizationUtils::getInstance();

    // initialize mujoco stuff
    //if (!mujoco_ros_control.init(nh_))
    //{
    //    RCLCPP_ERROR(this->get_logger(), "Could not initialise mujoco.");
    //    return 1;
    //}

    // init GLFW
    if ( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // make context current
    glfwMakeContextCurrent(window);

    // initialize mujoco visualization functions
    mujoco_visualization_utils.init(mujoco_ros2_control_plugin.mujoco_model_, mujoco_ros2_control_plugin.mujoco_data_, window);

    // spin
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    rclcpp::spin(node);

    // run main loop, target real-time simulation and 60 fps rendering
    while ( rclcpp::ok() && !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        mjtNum sim_start = mujoco_ros2_control_plugin.mujoco_data_->time;
        while ( mujoco_ros2_control_plugin.mujoco_data_->time - sim_start < 1.0/60.0 && rclcpp::ok() )
        {
            mujoco_ros2_control_plugin.update();
        }
        mujoco_visualization_utils.update(window);
    }

    mujoco_visualization_utils.terminate();

    return 0;
}
