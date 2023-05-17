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
#include <algorithm>

#include "angles/angles.h"

#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mujoco_ros2_control
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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

        /**
         * Load and stores the required Datas and Pointers from MuJoCo and ROS2 to this class
         * @param model_nh          ROS2 Node Handle
         * @param mujoco_model      Pointer to the MuJoCo model
         * @param mujoco_data       Pointer to the MuJoCo data
         * @param hardware_info     Description of the ROS2 Control definitions
         * @param urdf_model_ptr    Pointer to parsed URDF model
         * @param objects_in_scene
         * @return
         */
        bool initSim(
                rclcpp::Node::SharedPtr & model_nh,
                mjModel* mujoco_model, mjData *mujoco_data,
                const hardware_interface::HardwareInfo & hardware_info,
                const urdf::Model *urdf_model_ptr) override;

        // Methods used to control a joint.
        enum ControlMethod {EFFORT, POSITION, VELOCITY};

        struct JointData
        {
            std::string name;
            int type;
            double lower_limit = 0.0;
            double upper_limit = 0.0;
            double velocity_limit = 2.0;
            double effort_limit = 0.0;
            std::vector<ControlMethod> control_methods;
            double position;
            double velocity;
            double effort;
            double effort_command;
            double position_command;
            double velocity_command;
            std::map<ControlMethod, int> actuators;
            std::vector<hardware_interface::CommandInterface*> command_interfaces;
            std::vector<hardware_interface::StateInterface*> state_interfaces;
            int mujoco_joint_id;
            int mujoco_qpos_addr;
            int mujoco_dofadr;
        };

        struct MimicJoint {
            std::size_t joint_index;
            std::size_t mimicked_joint_index;
            double multiplier = 1.0;
        };

    private:
        void registerJoints(const hardware_interface::HardwareInfo & hardware_info,
                            const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints);
        static bool extractSubstr(std::string const & str, std::string const & ending, std::string& joint_name);


        /// \brief Degrees od freedom.
        size_t n_dof_;

        /// \brief Mujoco Model Ptr.
        mjModel *mujoco_model_;
        mjData *mujoco_data_;

        /// \brief last time the write method was called.
        rclcpp::Time last_update_sim_time_ros_;

        /// \brief state interfaces that will be exported to the Resource Manager
        std::vector<hardware_interface::StateInterface> state_interfaces_;

        /// \brief command interfaces that will be exported to the Resource Manager
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        /// \brief mapping of mimicked joints to index of joint they mimic
        std::vector<MimicJoint> mimic_joints_;

    protected:
        std::map<std::string, JointData> joints_;
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
