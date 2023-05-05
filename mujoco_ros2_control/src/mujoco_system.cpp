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

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "mujoco_ros2_control/mujoco_system.hpp"
//#include "gazebo/sensors/ImuSensor.hh"
//#include "gazebo/sensors/ForceTorqueSensor.hh"
//#include "gazebo/sensors/SensorManager.hh"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"



namespace mujoco_ros2_control {

    bool MujocoSystem::initSim(
            rclcpp::Node::SharedPtr &model_nh,
            mjModel *mujoco_model, mjData *mujoco_data,
            const hardware_interface::HardwareInfo &hardware_info,
            const urdf::Model *const urdf_model,
            uint objects_in_scene) {

        this->nh_ = model_nh;
        this->urdf_model_ = urdf_model;
        this->mujoco_model_ = mujoco_model;
        this->mujoco_data_ = mujoco_data;
        this->n_dof_ = mujoco_model_->njnt;
        RCLCPP_INFO(this->nh_->get_logger(), "%zu robot degrees of freedom found.", n_dof_);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i generalized coordinates (qpos) found.", mujoco_model_->nq);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i degrees of freedom (qvel) found.", mujoco_model_->nv);
        RCLCPP_INFO(this->nh_->get_logger(), "%i actuators/controls (ctrl) found.", mujoco_model_->nu);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i actuation states (act) found.", mujoco_model_->na);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i joints (njnt) found.", mujoco_model_->njnt);
        registerJoints(hardware_info);
        //registerSensors(hardware_info, mujoco_model, mujoco_data);

        return true;
    }

    void MujocoSystem::registerJoints(const hardware_interface::HardwareInfo &hardware_info) {

        for (int mujoco_joint_id = 0; mujoco_joint_id < n_dof_; mujoco_joint_id++) {
            std::string joint_name = mj_id2name(mujoco_model_, mjOBJ_JOINT, mujoco_joint_id);
            MujocoJointData mj_joint_data{};
            mj_joint_data.id = mujoco_joint_id;
            mj_joint_data.qpos_addr = mujoco_model_->jnt_qposadr[mujoco_joint_id];
            mj_joint_data.qvel_addr = mujoco_model_->jnt_dofadr[mujoco_joint_id];
            mj_joint_data.type = mujoco_model_->jnt_type[mujoco_joint_id];
            mujoco_joints_.insert(std::pair<std::string, MujocoJointData>(joint_name, mj_joint_data));
        }
        for (auto& mujoco_joint : mujoco_joints_) {
            RCLCPP_DEBUG(this->nh_->get_logger(), "%s: %s", mujoco_joint.first.c_str(), mujoco_joint.second.to_string().c_str());
        }
        for (int mujoco_actuator_id = 0; mujoco_actuator_id < mujoco_model_->nu; mujoco_actuator_id++) {
            std::string actuator_name = mj_id2name(mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);
            MujocoActuatorData mj_actuator_data{};
            mj_actuator_data.id = mujoco_actuator_id;
            mujoco_actuators_.insert(std::pair<std::string, MujocoActuatorData>(actuator_name, mj_actuator_data));
        }

        for (auto& mujoco_actuator : mujoco_actuators_) {
            RCLCPP_DEBUG(this->nh_->get_logger(), "%s: %s", mujoco_actuator.first.c_str(), mujoco_actuator.second.to_string().c_str());
        }

        auto get_initial_value = [this](const hardware_interface::InterfaceInfo &interface_info) {
            if (!interface_info.initial_value.empty()) {
                double value = std::stod(interface_info.initial_value);
                RCLCPP_DEBUG(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
                return value;
            } else {
                return 0.0;
            }
        };

        for (auto& joint_info : hardware_info.joints) {
            joints_.insert(std::pair<std::string, JointData>(joint_info.name, JointData()));
            JointData& joint = joints_.at(joint_info.name);
            joint.name = joint_info.name;
            joint.mujoco_joint_id = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint.name.c_str());
            joint.mujoco_qpos_addr = mujoco_model_->jnt_qposadr[joint.mujoco_joint_id];
            joint.mujoco_qvel_addr = mujoco_model_->jnt_dofadr[joint.mujoco_joint_id];
            if (joint.mujoco_joint_id == -1)
            {
                RCLCPP_WARN(this->nh_->get_logger(), "Joint %s not found in Mujoco model!", joint.name.c_str());
            }

            for(auto& state_interface : joint_info.state_interfaces) {
                if (state_interface.name == "position") {
                    joint.position = get_initial_value(state_interface);
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position));
                } else if (state_interface.name == "velocity") {
                    joint.velocity = get_initial_value(state_interface);
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity));
                } else if (state_interface.name == "effort") {
                    joint.effort = std::max(1.0, get_initial_value(state_interface));
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort));
                }
            }

            for(auto& command_interface : joint_info.command_interfaces) {
                if (command_interface.name == "position") {
                    joint.position = get_initial_value(command_interface);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position_command));
                } else if (command_interface.name == "velocity") {
                    joint.velocity = get_initial_value(command_interface);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity_command));
                } else if (command_interface.name == "effort") {
                    joint.effort = get_initial_value(command_interface);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort_command));
                }
            }
        }
    }

    CallbackReturn
    MujocoSystem::on_init(const hardware_interface::HardwareInfo &system_info) {
        if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    MujocoSystem::export_state_interfaces() {
        return std::move(this->state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface>
    MujocoSystem::export_command_interfaces() {
        return std::move(this->command_interfaces_);
    }

    CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MujocoSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    MujocoSystem::perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) {

        for (auto& joint : joints_) {
            for (const std::string &interface_name : stop_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION) {
                    joint.second.control_method &= static_cast<ControlMethod_>(VELOCITY & EFFORT);
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    joint.second.control_method &= static_cast<ControlMethod_>(POSITION & EFFORT);
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    joint.second.control_method &= static_cast<ControlMethod_>(POSITION & VELOCITY);
                }
            }
            for (const std::string &interface_name : start_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION) {
                    joint.second.control_method |= POSITION;
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    joint.second.control_method |= VELOCITY;
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    joint.second.control_method |= EFFORT;
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto& joint_item : joints_) {
            JointData& joint = joint_item.second;
            // TODO: Resolve the hardcoded stuff
            if (string_ends_with(joint.name, "FJ1") || string_ends_with(joint.name, "FJ2")) {
                std::string actuator_name = joint.name;
                actuator_name[actuator_name.size() - 1] = '0';
                MujocoActuatorData& actuator = mujoco_actuators_.at(actuator_name);
                joint.effort = mujoco_data_->qfrc_actuator[actuator.id]/2;
            } else {
                joint.effort = mujoco_data_->qfrc_applied[joint.mujoco_qvel_addr];
            }
            if (joint.type == urdf::Joint::PRISMATIC) {
                joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
            } else {
                joint.position += angles::shortest_angular_distance(joint.position, mujoco_data_->qpos[joint.mujoco_qpos_addr]);
            }
            joint.velocity = mujoco_data_->qvel[joint.mujoco_qvel_addr];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto& actuator : mujoco_actuators_) {
            if (joints_[actuator.first].control_method & POSITION) {
                mujoco_data_->ctrl[actuator.second.id] = joints_[actuator.first].position_command;
                RCLCPP_INFO(this->nh_->get_logger(), "mujococ_data_position: %f", mujoco_data_->ctrl[actuator.second.id]);
            } else if (joints_[actuator.first].control_method & VELOCITY) {
                mujoco_data_->ctrl[actuator.second.id] = joints_[actuator.first].velocity_command;
                RCLCPP_INFO(this->nh_->get_logger(), "mujococ_data_velocity: %f", mujoco_data_->ctrl[actuator.second.id]);
            } else if (joints_[actuator.first].control_method & EFFORT) {
                mujoco_data_->ctrl[actuator.second.id] = joints_[actuator.first].effort_command;
                RCLCPP_INFO(this->nh_->get_logger(), "mujococ_data_effort: %f", mujoco_data_->ctrl[actuator.second.id]);
            }

            /**if (string_ends_with(actuator.first, "FJ0")) {
                RCLCPP_INFO(this->nh_->get_logger(), "%s", actuator.first.c_str());
                std::string joint_1_name = actuator.first;
                std::string joint_2_name = actuator.first;
                joint_1_name[joint_1_name.size() - 1] = '1';
                joint_2_name[joint_2_name.size() - 1] = '2';
                JointData& joint_1 = joints_.at(joint_1_name);
                JointData& joint_2 = joints_.at(joint_2_name);
                if (joint_1.control_method & POSITION) {
                    mujoco_data_->ctrl[actuator.second.id] = joint_1.position_command + joint_2.position_command;
                } else if (joint_1.control_method & VELOCITY) {
                    mujoco_data_->ctrl[actuator.second.id] = joint_1.velocity_command + joint_2.velocity_command;
                } else if (joint_1.control_method & EFFORT) {
                    mujoco_data_->ctrl[actuator.second.id] = joint_1.effort_command + joint_2.effort_command;
                }
            }*/
        }

        return hardware_interface::return_type::OK;
    }

    bool MujocoSystem::string_ends_with(const std::string &value, const std::string &ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(
        mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)
