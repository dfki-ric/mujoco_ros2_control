#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP

// std libraries
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <ctime>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// Mujoco dependencies
#include "mujoco/mujoco.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

// ros_control
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// msgs
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// URDF
#include "urdf/urdf/model.h"

// Package header
#include "mujoco_ros2_control/mujoco_visualization.hpp"
#include "mujoco_ros2_control/mujoco_system.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace mujoco_ros2_control
{
class MujocoRos2Control {

public:
    /**
     * Set up the ros node, the controller_manager and mujoco
     * @param node pointer to a ros node
     */
    explicit MujocoRos2Control(rclcpp::Node::SharedPtr &node);
    /**
     * Destructor stop and join the controller_manager executor and remove and deactivate mujoco
     */
    virtual ~MujocoRos2Control();

    // step update function
    void update();

protected:
    /**
     * Publish the clock message to ros
     */
    void publish_sim_time();

    /**
     * Initialize the controller manager
     */
    void init_controller_manager();

    /**
     * Initialize the MuJoCo model and create the MuJoCo data
     */
    void init_mujoco();

    // pointer to the mujoco model
    mjModel* mujoco_model_{};
    // pointer to the mujoco data
    mjData* mujoco_data_{};

    // Visualization class
    mujoco_visualization::MujocoVisualization& mj_vis_ = mujoco_visualization::MujocoVisualization::getInstance();

    // node handles
    std::shared_ptr<rclcpp::Node> nh_;

    // Controller manager related pointers
    // Executor to spin the controller
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    // Recourse manager for child nodes
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
    // bool to stop the controller_manager node before thread is joined
    bool stop_{};
    // Thread where the executor will spin
    std::thread thread_executor_spin_;
    // interface loader
    std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> > robot_hw_sim_loader_;
    // robot simulator interface
    std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_;
    // controller manager node
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // realtime_tools publisher for the clock message
    using ClockPublisher = realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>;
    using ClockPublisherPtr = std::unique_ptr<ClockPublisher>;
    //Pointer to the default ros clock publisher
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    //Pointer to the realtime_tools ros clock publisher
    ClockPublisherPtr clock_publisher_;


    // timing and durations
    // Period from ros2_control
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);
    // Period from MuJoCo
    rclcpp::Duration mujoco_period_ = rclcpp::Duration(1, 0);
    // timestamp of last controller manager update
    rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);

    // MuJoCo simulation time after first step
    double mujoco_start_time_;
    // Timestamp of System time
    struct timespec startTime_, currentTime_;
    // frequency of ros clock publisher
    double pub_clock_frequency_;
    // last clock publish timestamp (in sim time)
    double last_pub_clock_time_;

    // Parameters
    // Speed factor for mujoco
    double real_time_factor_;
    // Show the mujoco gui
    bool show_gui_;
};
}  // namespace mujoco_ros2_control


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
