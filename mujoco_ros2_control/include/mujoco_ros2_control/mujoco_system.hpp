// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "mujoco_ros2_control/mujoco_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "control_toolbox/pid.hpp"

namespace mujoco_ros2_control
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
    class MujocoSystemPrivate;

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

    class MujocoSystem : public MujocoSystemInterface
    {
    public:
        // Documentation Inherited
        CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info)
        override;

        // Documentation Inherited
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Documentation Inherited
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Documentation Inherited
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        // Documentation Inherited
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        // Documentation Inherited
        hardware_interface::return_type perform_command_mode_switch(
                const std::vector<std::string> & start_interfaces,
                const std::vector<std::string> & stop_interfaces) override;

        // Documentation Inherited
        hardware_interface::return_type read(
                const rclcpp::Time & time,
                const rclcpp::Duration & period) override;

        // Documentation Inherited
        hardware_interface::return_type write(
                const rclcpp::Time & time,
                const rclcpp::Duration & period) override;

        // Documentation Inherited
        bool initSim(
                rclcpp::Node::SharedPtr & model_nh,
                mjModel* mujoco_model, mjData *mujoco_data,
                const hardware_interface::HardwareInfo & hardware_info,
                const urdf::Model *urdf_model,
                uint objects_in_scene) override;



        // Methods used to control a joint.
        //enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

        struct JointData
        {
            std::string name;
            int type;
            double lower_limit;
            double upper_limit;
            double effort_limit;
            ControlMethod control_method;
            control_toolbox::Pid pid_controller;
            double position;
            double velocity;
            double effort;
            double effort_command;
            double position_command;
            double last_position_command;
            double velocity_command;
            hardware_interface::CommandInterface* command_interfaces;
            hardware_interface::StateInterface* state_interfaces;
            int mujoco_joint_id;
            int mujoco_qpos_addr;
            int mujoco_qvel_addr;

            std::string to_string()
            {
                std::stringstream ss;
                ss << "Joint " << name << " has type " << type << ", mujoco addresses " << mujoco_joint_id << ", " <<
                   mujoco_qpos_addr << ", " << mujoco_qvel_addr << ".\nJoint status: p:" << position << " v:" << velocity <<
                   " e:" << effort << "\nJoint position address: " << &position;
                return ss.str();
            }
        };

        struct MujocoJointData
        {
            int id;
            int qpos_addr;
            int qvel_addr;
            int type;

            std::string to_string()
            {
                std::stringstream ss;
                ss << "Mujoco Joint has type " << type << ", mujoco addresses " << id << ", " <<
                   qpos_addr << ", " << qvel_addr;
                return ss.str();
            }
        };

        struct MujocoActuatorData
        {
            int id;

            std::string to_string()
            {
                std::stringstream ss;
                ss << "Mujoco Actuator has mujoco address " << id << ".";
                return ss.str();
            }
        };

    private:
        void registerJoints(const hardware_interface::HardwareInfo & hardware_info);

        static bool string_ends_with(std::string const & value, std::string const & ending);

        //void registerSensors(
        //        const hardware_interface::HardwareInfo & hardware_info,
        //        mjModel* mujoco_model, mjData *mujoco_data);

        std::vector<hardware_interface::CommandInterface> command_interfaces_;
        std::vector<hardware_interface::StateInterface> state_interfaces_;

    protected:
        unsigned int n_dof_;
        //std::map<std::string, JointData> joints_;
        //std::map<std::string, MujocoJointData> mujoco_joints_;
        std::map<std::string, MujocoActuatorData> mujoco_actuators_;

        std::unique_ptr<MujocoSystemPrivate> dataPtr;
        // mujoco elements
        mjModel* mujoco_model_;
        mjData* mujoco_data_;
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_