/**
 * @file mujoco_system.hpp
 * @brief This file contains the implementation of the MujocoSystem class.
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
 *
 * This code is a modified version of the original code from Open Source Robotics Foundation, Inc.
 * https://github.com/ros-controls/gazebo_ros2_control/blob/master/gazebo_ros2_control/include/gazebo_ros2_control/gazebo_system.hpp
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


#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

// std libraries
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

// Mujoco system interface
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

// ROS Hardware Interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// ROS messages
#include "std_msgs/msg/bool.hpp"

namespace mujoco_ros2_control {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @class MujocoSystem
     * @brief Implements the MujocoSystem interface for controlling a Mujoco robot model.
     *
     * The MujocoSystem class is an implementation of the MujocoSystemInterface, which serves as the interface between the
     * Mujoco physics engine and the ROS 2 control framework. It provides methods for initializing the Mujoco model,
     * updating the model state, and sending commands to the model.
     *
     * This class utilizes the Mujoco C library to interact with the Mujoco model. It provides functions for initializing
     * the model, setting the control inputs, and stepping the simulation forward in time. It also interfaces with the ROS 2
     * control framework by implementing the necessary interfaces and callbacks.
     *
     * The MujocoSystem class is responsible for reading the state of the Mujoco model, updating the controller outputs, and
     * writing the results back to the model. It acts as the bridge between the ROS 2 control system and the underlying
     * Mujoco simulation.
     */
    class MujocoSystem : public MujocoSystemInterface {
    public:
        /**
         * @brief Callback function for the on_init lifecycle event.
         *
         * This method is called when the system is initialized. It returns the success status of the initialization.
         *
         * @param system_info System information.
         * @return The success status of the on_init callback.
         */
        CallbackReturn on_init(const hardware_interface::HardwareInfo &system_info) override;

        /**
         * @brief Exports the state interfaces.
         *
         * This method exports the state interfaces by returning a vector of state interfaces.
         *
         * @return A vector of state interfaces.
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Exports the command interfaces.
         *
         * This method exports the command interfaces by returning a vector of command interfaces.
         *
         * @return A vector of command interfaces.
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Callback function for the on_activate lifecycle event.
         *
         * This method is called when the system is activated. It returns the success status of the activation.
         *
         * @param previous_state The previous lifecycle state.
         * @return The success status of the on_activate callback.
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Callback function for the on_deactivate lifecycle event.
         *
         * This method is called when the system is deactivated. It returns the success status of the deactivation.
         *
         * @param previous_state The previous lifecycle state.
         * @return The success status of the on_deactivate callback.
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;


        /**
         * @brief Performs the command mode switch for the interfaces.
         *
         * This method performs the command mode switch by updating the control methods of the joints based on the
         * start and stop interfaces.
         *
         * @param start_interfaces The interfaces to start.
         * @param stop_interfaces The interfaces to stop.
         * @return The return type of the command mode switch.
         */
        hardware_interface::return_type perform_command_mode_switch(
                const std::vector<std::string> &start_interfaces,
                const std::vector<std::string> &stop_interfaces) override;

        /**
         * @brief Reads the joint states from the Mujoco simulation.
         *
         * This method reads the joint states from the Mujoco simulation and updates the joint positions, velocities,
         * and efforts.
         *
         * @param time The current time.
         * @param period The duration of the current cycle.
         * @return The return type of the read operation.
         */
        hardware_interface::return_type read(
                const rclcpp::Time &time,
                const rclcpp::Duration &period) override;

        /**
         * @brief Writes the commands to the Mujoco simulation.
         *
         * This method writes the commands to the Mujoco simulation by updating the joint control efforts.
         *
         * @param time The current time.
         * @param period The duration of the current cycle.
         * @return The return type of the write operation.
         */
        hardware_interface::return_type write(
                const rclcpp::Time &time,
                const rclcpp::Duration &period) override;

        /**
         * @brief Initializes the Mujoco simulation.
         *
         * This method initializes the Mujoco simulation by setting the Mujoco model and data pointers,
         * registering joints, and returning true if successful.
         *
         * @param mujoco_model A pointer to the Mujoco model.
         * @param mujoco_data A pointer to the Mujoco data.
         * @param hardware_info Hardware information.
         * @param urdf_model_ptr A pointer to the URDF model.
         * @return True if initialization is successful, false otherwise.
         */
        bool initSim(
                mjModel *mujoco_model, mjData *mujoco_data,
                const hardware_interface::HardwareInfo &hardware_info,
                const urdf::Model *urdf_model_ptr) override;

    private:

        /**
         * @brief Registers the joints in the Mujoco simulation.
         *
         * This method registers the joints in the Mujoco simulation by creating a JointData struct for each joint,
         * setting the necessary joint information, and populating the joint limits.
         *
         * @param hardware_info Hardware information.
         * @param joints A map of joint names to URDF joint pointers.
         */
        void registerJoints(const hardware_interface::HardwareInfo &hardware_info,
                            const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints);

        // Variables
        mjModel *mujoco_model_;  ///< Pointer to the Mujoco model.
        mjData *mujoco_data_;    ///< Pointer to the Mujoco data.

        // Enums
        /**
         * @brief Enum representing control methods for a joint.
         *
         * This enum defines the available control methods that can be used to control a joint.
         * It provides options for different control strategies, such as position control, velocity control, etc.
         */
        enum ControlMethod {
            EFFORT,  ///< Effort control method.
            POSITION,  ///< Position control method.
            VELOCITY,  ///< Velocity control method.
            ACCELERATION ///< Acceleration control method.
        };

        // Structs
        /**
         * @brief Struct representing the data for a pid controller of a joint
         */
        struct PIDConfig {
            double kp{0.0}; ///< Proportional Gain
            double ki{0.0}; ///< Integral Gain
            double kd{0.0}; ///< Derivative Gain
            double kvff{0.0};  ///< Velocity Feedforward Gain
            double kaff{0.0};  ///< Acceleration Feedforward Gain
            double integral{0.0}; ///< Actual integral value
            double prev_error{0.0}; ///< Previous error
            bool position{false}; ///< Was tau calculated for position command
            bool velocity{false}; ///< Was tau calculated for velocity command
        };

        /**
         * @brief Struct representing joint data.
         *
         * This struct contains information related to a joint, including its name, limits, control methods,
         * current state, commanded values, actuator mappings, interfaces, and Mujoco-specific IDs.
         */
        struct JointData {
            std::string name;  ///< Name of the joint.
            double lower_limit = std::numeric_limits<double>::min();  ///< Lower limit of the joint position.
            double upper_limit = std::numeric_limits<double>::max();  ///< Upper limit of the joint position.
            double velocity_limit = std::numeric_limits<double>::max();  ///< Limit on the joint velocity.
            double acceleration_limit = std::numeric_limits<double>::max();  ///< Limit on the joint acceleration.
            double effort_limit = std::numeric_limits<double>::max();  ///< Limit on the joint effort.
            std::vector<ControlMethod> control_methods;  ///< Available control methods for the joint.
            double position;  ///< Current position of the joint.
            double velocity;  ///< Current velocity of the joint.
            double acceleration;  ///< Current effort applied to the joint.
            double effort;  ///< Current effort applied to the joint.
            double position_command;  ///< Commanded position for the joint.
            double velocity_command;  ///< Commanded velocity for the joint.
            double acceleration_command;  ///< Commanded acceleration for the joint.
            double effort_command;  ///< Commanded effort to be applied to the joint.
            double last_command; ///< Last command;
            std::map<ControlMethod, int> actuators;  ///< Mapping of control methods to actuator IDs.
            std::vector<hardware_interface::CommandInterface *> command_interfaces;  ///< Command interfaces associated with the joint.
            std::vector<hardware_interface::StateInterface *> state_interfaces;  ///< State interfaces associated with the joint.
            int mujoco_joint_id;  ///< ID of the joint in the Mujoco simulation.
            int mujoco_qpos_addr;  ///< Address of the joint position in the Mujoco data structure.
            int mujoco_dofadr;  ///< Degree-of-freedom (DOF) address of the joint in the Mujoco data structure.
            int type; ///< Type of the joint
            PIDConfig pid; ///< Gains for pid control when input command is position or velocity
        };

        /**
         * @brief This struct holds information about mimic joints
         */
        struct MimicJoint {
            std::string joint;
            std::string mimiced_joint;
            double mimic_multiplier = 0.0;
            double mimic_offset = 0.0;

        };

        /**
         * @brief Last update simulation time in ROS.
         *
         * This variable represents the time of the last update in the ROS simulation. It is used to track the timing
         * of the simulation and synchronize it with other components or systems.
         */
        rclcpp::Time last_update_sim_time_ros_;


        /**
         * @brief State interfaces for the joint.
         *
         * This vector holds pointers to the state interfaces associated with the joint. State interfaces are used to
         * provide access to the joint's current state, such as position, velocity, and effort, for external systems or modules.
         * It allows read-only access to the joint's state information.
         */
        std::vector<hardware_interface::StateInterface> state_interfaces_;

        /**
         * @brief Command interfaces for the joint.
         *
         * This vector contains pointers to the command interfaces associated with the joint. Command interfaces are used
         * to send control commands, such as position, velocity, or effort commands, to the joint. It allows external systems
         * or modules to control the joint's behavior.
         */
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        double pid_control(double kp, double ki, double kd, double error, double last_error, double dt);

    protected:
        std::map<std::string, JointData> joints_; ///< Map of joint names to JointData structs.
        std::vector<MimicJoint> mimiced_joints_; ///!< Mimiced Joints
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
