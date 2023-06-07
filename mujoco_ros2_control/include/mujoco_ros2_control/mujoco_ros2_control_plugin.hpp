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

// Mujoco dependencies
#include "mujoco/mujoco.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco_ros2_control/mujoco_system.hpp"

// ros_control
#include "mujoco_ros2_control/mujoco_system.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"
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


#include "mujoco_ros2_control/mujoco_visualization.hpp"


#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

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

    // Visualization
    mujoco_visualization::MujocoVisualization& mj_vis_ = mujoco_visualization::MujocoVisualization::getInstance();

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

    // robot simulator interface
    std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_;

    // controller manager
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // simulated clock publisher
    using ClockPublisher = realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>;
    using ClockPublisherPtr = std::unique_ptr<ClockPublisher>;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    ClockPublisherPtr clock_publisher_;

    double pub_clock_frequency_ = 0;
    double last_pub_clock_time_;
    // timing
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);
    rclcpp::Duration mujoco_period_ = rclcpp::Duration(1, 0);
    rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
    rclcpp::Time last_write_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);

    std::chrono::system_clock::time_point system_start_time_;
    double mujoco_start_time_;
    bool show_gui_;
    struct timespec startTime_, currentTime_;

    double simulation_frequency_;
    double real_time_factor_;
};
}  // namespace mujoco_ros2_control


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
