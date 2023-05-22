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
#include "rclcpp/executors/multi_threaded_executor.hpp"

// Mujoco dependencies
#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

#include "yaml-cpp/yaml.h"

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
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// openGL stuff
#include <GLFW/glfw3.h>
#include <mujoco_ros2_control/mujoco_visualization_utils.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

namespace mujoco_ros2_control
{
class MujocoRos2Control {

public:
    explicit MujocoRos2Control(rclcpp::Node::SharedPtr &node);
    virtual ~MujocoRos2Control();

    // step update function
    void update();

    // pointer to the mujoco model
    mjModel* mujoco_model_{};
    mjData* mujoco_data_{};

protected:

    // get the URDF XML from the parameter server
    [[nodiscard]] std::string get_urdf(const std::string& param_name) const;

    // publish simulation time to ros clock
    void publish_sim_time();

    // node handles
    std::shared_ptr<rclcpp::Node> parameter_node_ = rclcpp::Node::make_shared("mujoco_param_node");
    std::shared_ptr<rclcpp::Node> model_node_;

    // Executor to spin the controller
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

    bool stop_{};

    // Thread where the executor will spin
    std::thread thread_executor_spin_;

    // interface loader
    std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> > robot_hw_sim_loader_;

    // strings
    std::string robot_namespace_;
    std::string robot_description_param_;
    std::string robot_description_node_;
    std::string robot_model_path_;

    // transmissions in this plugin's scope
    //std::vector<transmission_interface::TransmissionInfo> transmissions_;

    // robot simulator interface
    std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_;

    // controller manager
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // simulated clock
    std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> pub_clock_;
    int pub_clock_frequency_{};
    rclcpp::Time last_pub_clock_time_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);

    // timing
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);
    rclcpp::Duration mujoco_period_ = rclcpp::Duration(1, 0);
    rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
    rclcpp::Time last_write_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
};
}  // namespace mujoco_ros2_control


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
