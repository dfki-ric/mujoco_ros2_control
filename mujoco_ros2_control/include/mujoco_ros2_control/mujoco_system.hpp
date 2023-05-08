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
#include "control_toolbox/pid.hpp"
//#include "joint_limits/joint_limits.hpp"
//#include "joint_limits/joint_limits_rosparam.hpp"
#include "std_msgs/msg/bool.hpp"

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
                const urdf::Model *urdf_model_ptr,
                uint objects_in_scene) override;



        // Methods used to control a joint.
        enum ControlMethod {NONE, EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

        struct JointData
        {
            std::string name;
            int type;
            double lower_limit = 0.0;
            double upper_limit = 0.0;
            double velocity_limit = 2.0;
            double effort_limit = 0.0;
            ControlMethod control_method;
            control_toolbox::Pid pid_controller;
            bool use_pid_controller;
            double position;
            double velocity;
            double effort;
            double effort_command;
            double position_command;
            double velocity_command;
            double last_position_command;

            std::vector<hardware_interface::CommandInterface*> command_interfaces;
            std::vector<hardware_interface::StateInterface*> state_interfaces;
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
            std::string joint_name;
            ControlMethod control_method;

            std::string to_string()
            {
                std::stringstream ss;
                ss << "Mujoco Actuator has mujoco address " << id << ".";
                return ss.str();
            }
        };

        struct MimicJoint {
            std::size_t joint_index;
            std::size_t mimicked_joint_index;
            double multiplier = 1.0;
        };

    private:
        void registerJoints(const hardware_interface::HardwareInfo & hardware_info,
                            const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints);

        static bool string_ends_with(std::string const & value, std::string const & ending);
        static bool extractSubstr(std::string const & str, std::string const & ending, std::string& joint_name);
        //void registerSensors(
        //        const hardware_interface::HardwareInfo & hardware_info,
        //        mjModel* mujoco_model, mjData *mujoco_data);


        /// \brief Degrees od freedom.
        size_t n_dof_;

        /// \brief Number of sensors.
        //size_t n_sensors_;

        /// \brief Mujoco Model Ptr.
        mjModel *mujoco_model_;
        mjData *mujoco_data_;
        const urdf::Model *urdf_model_;

        /// \brief last time the write method was called.
        rclcpp::Time last_update_sim_time_ros_;

        /// \brief vector with the joint's names.
        //std::vector<std::string> joint_names_;

        /// \brief vector with the control method defined in the URDF for each joint.
        //std::vector<MujocoSystemInterface::ControlMethod> joint_control_methods_;

        std::vector<control_toolbox::Pid> pid_controllers_;

        /// \brief handles to the joints from within Mujoco
        //std::vector<MujocoSystem::MujocoJointData> sim_joints_;

        /// \brief vector with the current joint position
        //std::vector<double> joint_position_;

        /// \brief vector with the current joint velocity
        //std::vector<double> joint_velocity_;

        /// \brief vector with the current joint effort
        //std::vector<double> joint_effort_;

        /// \brief vector with the current cmd joint position
        //std::vector<double> joint_position_cmd_;

        /// \brief vector with the current cmd joint velocity
        //std::vector<double> joint_velocity_cmd_;

        /// \brief vector with the current cmd joint effort
        //std::vector<double> joint_effort_cmd_;

        /// \brief vector with the lower joint limits
        std::vector<double> joint_lower_limit_;

        /// \brief vector with the upper joint limits
        std::vector<double> joint_upper_limit_;

        /// \brief vector with the effort limits
        std::vector<double> joint_effort_limit_;

        /**
        /// \brief handles to the imus from within Gazebo
        std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

        /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
        std::vector<std::array<double, 10>> imu_sensor_data_;

        /// \brief handles to the FT sensors from within Gazebo
        std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

        /// \brief An array per FT sensor for 3D force and torquee
        std::vector<std::array<double, 6>> ft_sensor_data_;
        */
        /// \brief state interfaces that will be exported to the Resource Manager
        std::vector<hardware_interface::StateInterface> state_interfaces_;

        /// \brief command interfaces that will be exported to the Resource Manager
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        /// \brief mapping of mimicked joints to index of joint they mimic
        std::vector<MimicJoint> mimic_joints_;

        /// \brief Gain which converts position error to a velocity command
        //double position_proportional_gain_;

    protected:
        std::map<std::string, JointData> joints_;
        std::map<std::string, MujocoJointData> mujoco_joints_;
        std::map<std::string, MujocoActuatorData> mujoco_actuators_;

    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
