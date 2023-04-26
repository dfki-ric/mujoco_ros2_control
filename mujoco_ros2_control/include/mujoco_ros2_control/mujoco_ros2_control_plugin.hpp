//
// Created by ubuntu22 on 18.04.23.
//

#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
//#include <pluginlib/class_loader.h>
#include <std_msgs/msg/bool.hpp>
//#include <ros/package.h>

// Mujoco dependencies
#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

// ros_control
#include <mujoco_ros2_control/mujoco_system.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>

// msgs
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mujoco_ros2_msgs/msg/model_states.hpp"

#include <controller_manager/controller_manager.hpp>
//#include <transmission_interface/transmission_parser.h>
#include <hardware_interface/hardware_info.hpp>

// openGL stuff
#include <GLFW/glfw3.h>
#include <mujoco_ros2_control/mujoco_visualization_utils.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

namespace mujoco_ros2_control
{
class MujocoRos2Control {

public:
    MujocoRos2Control(rclcpp::Node::SharedPtr &node);
    virtual ~MujocoRos2Control();

    // initialize params and controller manager
    bool init();

    // step update function
    void update();

    // pointer to the mujoco model
    mjModel* mujoco_model_{};
    mjData* mujoco_data_{};

    // number of degrees of freedom
    unsigned int n_dof_{};

    // number of free joints in simulation
    unsigned int n_free_joints_;

protected:
    // free or static object
    enum Object_State { STATIC = true, FREE = false };

    // get the URDF XML from the parameter server
    std::string get_urdf(const std::string& param_name) const;

    // setup initial sim environment
    void setup_sim_environment();

    // parse transmissions from URDF
    bool parse_transmissions(const std::string& urdf_string);

    // get number of degrees of freedom
    void get_number_of_dofs();

    // publish simulation time to ros clock
    void publish_sim_time();

    // check for free joints in the mujoco model
    void check_objects_in_scene();

    // publish free objects
    void publish_objects_in_scene();

    // transform type id to type name
    static std::string geom_type_to_string(int geom_id);

    // node handles
    std::shared_ptr<rclcpp::Node> parameter_node_;
    std::shared_ptr<rclcpp::Node> model_node_;

    // Executor to spin the controller
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

    bool stop_{};

    // Thread where the executor will sping
    std::thread thread_executor_spin_;

    // interface loader
    std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> > robot_hw_sim_loader_;

    // strings
    std::string robot_namespace_;
    std::string robot_description_param_;
    std::string robot_description_node_;
    std::string robot_model_path_;

    // vectors
    std::vector<int> mujoco_ids;
    //std::vector<int>::iterator it;
    std::vector<std::string> robot_link_names_;
    std::map<int, Object_State> objects_in_scene_;

    // transmissions in this plugin's scope
    //std::vector<transmission_interface::TransmissionInfo> transmissions_;

    // robot simulator interface
    std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_;

    // controller manager
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // simulated clock
    std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> pub_clock_;
    int pub_clock_frequency_{};
    rclcpp::Time last_pub_clock_time_;

    // timing
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);
    rclcpp::Duration mujoco_period_ = rclcpp::Duration(1, 0);
    rclcpp::Time last_update_sim_time_ros_;
    rclcpp::Time last_write_sim_time_ros_;

    // publishing
    std::shared_ptr<rclcpp::Publisher<mujoco_ros2_msgs::msg::ModelStates>> objects_in_scene_publisher;
};
}  // namespace mujoco_ros2_control


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
