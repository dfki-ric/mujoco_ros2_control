/**
* @file mujoco_ros2_control_plugin.hpp
*
* @brief This file contains the implementation of the Mujoco ROS2 Control plugin.
*
*
* @author Adrian Danzglock
* @date 2023
*
* @license GNU General Public License, version 3 (GPL-3.0)
* @copyright Copyright (c) 2023, DFKI GmbH
*
* This file is governed by the GNU General Public License, version 3 (GPL-3.0).
* The GPL-3.0 is a copyleft license that allows users to use, modify, and distribute software
* while ensuring that these freedoms are passed on to subsequent users. It requires that any
* derivative works or modifications of the software be licensed under the GPL-3.0 as well.
* You should have received a copy of the GNU General Public License along with this program.
* If not, see https://www.gnu.org/licenses/gpl-3.0.html.
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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// URDF
#include "urdf/urdf/model.h"

// Package header
#include "mujoco_ros2_control/mujoco_visualization.hpp"
#include "mujoco_ros2_control/mujoco_system.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "mujoco_ros2_control/mujoco_depth_camera.hpp"

namespace mujoco_ros2_control
{
/**
 * @class MujocoRos2Control
 * @brief Implements a ROS 2 control plugin for controlling a Mujoco robot model.
 *
 * The MujocoRos2Control class is a ROS 2 control plugin that provides integration between the Mujoco physics engine
 * and the ROS 2 control framework. It enables control of a Mujoco robot model using ROS 2 controllers and interfaces.
 *
 * This class handles the initialization of the Mujoco model, the controller manager, and the hardware interfaces. It
 * also manages the simulation loop, where the controllers are updated and the robot state is synchronized with the
 * Mujoco model.
 *
 * To use this plugin, it is necessary to provide the path to the Mujoco XML model file and the path to the ROS 2 control
 * parameters file. The plugin initializes the Mujoco model, creates the necessary hardware interfaces, loads the
 * controllers, and starts the simulation loop.
 *
 * The plugin supports real-time factor control to ensure that the simulation runs at the desired speed. It publishes
 * the simulation time as a ROS 2 clock message and provides access to the current time for synchronization with other
 * components.
 *
 * The MujocoRos2Control class is designed to be instantiated as a ROS 2 node, and it can be easily integrated into a
 * ROS 2 control system for controlling a Mujoco robot model.
 *
 * @note This class assumes that the necessary ROS 2 parameters, such as the robot model path, simulation frequency,
 * and controller update rate, are properly set before initialization.
 */
class MujocoRos2Control {

public:
    /**
     * @brief Constructs a `MujocoRos2Control` object.
     *
     * This constructor initializes a `MujocoRos2Control` object with the given ROS 2 node.
     *
     * @param node A shared pointer to the ROS 2 node that will be used for communication and control.
     */
    explicit MujocoRos2Control(rclcpp::Node::SharedPtr &node);
    /**
     * @brief Destroys the `MujocoRos2Control` object.
     *
     * This destructor cleans up any resources associated with the `MujocoRos2Control` object.
     * It is responsible for releasing any dynamically allocated memory or performing any necessary cleanup operations.
     * It is automatically called when the object goes out of scope or is explicitly deleted.
     */
    virtual ~MujocoRos2Control();

    /**
     * @brief Updates the state of the MujocoRos2Control.
     *
     * This method is responsible for updating the state of the `MujocoRos2Control` based on the current simulation time and
     * the duration of the simulation step. It performs the necessary computations and updates the internal variables and
     * components of the `MujocoRos2Control`, including publishing the simulation time, updating the controllers, and
     * updating the Mujoco model.
     */
    void update();

private:
    /**
     * @brief Publishes the current simulation time.
     *
     * This method publishes the current simulation time to the `/clock` topic. It checks the publishing frequency to ensure
     * that the time is published at the desired rate. If the elapsed time since the last publication is less than the
     * reciprocal of the publishing frequency, the method returns without publishing.
     */
    void publish_sim_time();

    /**
     * @brief Initializes the controller manager and hardware interfaces.
     *
     * This method initializes the controller manager and sets up the hardware interfaces for communication between the
     * controllers and the Mujoco model. It loads the necessary resources, creates instances of the
     * MujocoSystemInterface, and sets up the controller manager.
     *
     * @note If any errors occur during the initialization process, appropriate error messages are logged, and the method returns.
     */
    void init_controller_manager();

    /**
     * @brief Initializes the Mujoco simulation environment.
     *
     * This method initializes the Mujoco simulation environment by loading the Mujoco model, setting the simulation frequency,
     * and creating the corresponding data structure. It also retrieves the Mujoco simulation period and stores it as a ROS duration.
     *
     * @note If an error occurs during the initialization process, the method logs a fatal error message and returns.
     */
    void init_mujoco();

    std::shared_ptr<rclcpp::Node> nh_; ///< ROS2 node handle

    // realtime_tools publisher for the clock message
    using ClockPublisher = realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>;
    using ClockPublisherPtr = std::unique_ptr<ClockPublisher>;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_; ///< Publisher for the Clock message
    ClockPublisherPtr clock_publisher_; ///< Clock publisher object
    double pub_clock_frequency_; ///< Frequency the Clock is published
    double last_pub_clock_time_; ///< Timestamp the clock was published

    // Mujoco-related variables
    mjModel* mujoco_model_{}; ///< Pointer to the Mujoco model
    mjData* mujoco_data_{}; ///< Pointer to the Mujoco data
    double mujoco_start_time_; ///< Start time of the Mujoco simulation
    struct timespec startTime_; ///< Start time for the real-time clock
    rclcpp::Duration mujoco_period_ = rclcpp::Duration(1, 0); ///< Update period of the mujoco simulation
    rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME); ///< Timestamp of the last update call
    double real_time_factor_; ///< Realtime factor of the simulation
    bool show_gui_; ///< Flag if the gui is loaded

    // Controller Manager
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_; ///< Resource manager for hardware interfaces
    rclcpp::executors::MultiThreadedExecutor::SharedPtr controller_manager_executor_; ///< Executor for controller manager
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_; ///< Controller manager object
    rclcpp::Duration control_period_ = rclcpp::Duration(1, 0); ///< Control period of the controller manager
    bool stop_{}; ///< Flag to stop the execution of the controller manager
    std::thread controller_manager_thread_executor_spin_; ///< Thread for the controller manager executor

    // Visualization class
    mujoco_visualization::MujocoVisualization& mj_vis_ = mujoco_visualization::MujocoVisualization::getInstance(); ///< MuJoCo visualizer object

    // interface loader
    std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> > robot_hw_sim_loader_; ///< Plugin loader for RobotHWSimInterface
    std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_; ///< Robot hardware simulation interface

    // Camera
    rclcpp::executors::MultiThreadedExecutor camera_executor_;
    std::thread camera_executor_thread_;
    rclcpp::Node::SharedPtr camera_node_;
    std::shared_ptr<mujoco_sensors::MujocoDepthCamera> camera_;
    std::vector<std::shared_ptr<mujoco_sensors::MujocoDepthCamera>> cameras_; ///< Cameras vector
    void registerSensors();


};
}  // namespace mujoco_ros2_control


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_CONTROL_PLUGIN_HPP
