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
            const urdf::Model *const urdf_model_ptr,
            uint objects_in_scene) {

        this->nh_ = model_nh;
        //this->urdf_model_ = urdf_model;
        this->mujoco_model_ = mujoco_model;
        this->mujoco_data_ = mujoco_data;
        this->n_dof_ = mujoco_model_->njnt;
        RCLCPP_INFO(this->nh_->get_logger(), "%zu robot degrees of freedom found.", n_dof_);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i generalized coordinates (qpos) found.", mujoco_model_->nq);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i degrees of freedom (qvel) found.", mujoco_model_->nv);
        RCLCPP_INFO(this->nh_->get_logger(), "%i actuators/controls (ctrl) found.", mujoco_model_->nu);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i actuation states (act) found.", mujoco_model_->na);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i joints (njnt) found.", mujoco_model_->njnt);
        registerJoints(hardware_info, urdf_model_ptr->joints_);
        //registerSensors(hardware_info, mujoco_model, mujoco_data);

        return true;
    }

    void MujocoSystem::registerJoints(const hardware_interface::HardwareInfo &hardware_info, const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints) {
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
            //MujocoActuatorData mj_actuator_data{};
            //mj_actuator_data.id = mujoco_actuator_id;
            std::string joint_name;
            //get actuators with their control method (the actuators must follow the naming convention)
            if (extractSubstr(actuator_name, "_position", joint_name)) {
                //mj_actuator_data.control_method = POSITION;
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(POSITION, mujoco_actuator_id));
                //mujoco_model_->actuator_gainprm[mujoco_actuator_id*mjNGAIN] = 0.1;
                //mujoco_model_->actuator_biasprm[mujoco_actuator_id*mjNBIAS] = -0.1;
            } else if (extractSubstr(actuator_name, "_velocity", joint_name)) {
                //mj_actuator_data.control_method = VELOCITY;
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(VELOCITY, mujoco_actuator_id));
                //mujoco_model_->actuator_gainprm[mujoco_actuator_id*mjNGAIN] = 1.0;
                //mujoco_model_->actuator_biasprm[mujoco_actuator_id*mjNBIAS] = -1.0;
            } else if (extractSubstr(actuator_name, "_effort", joint_name)) {
                //mj_actuator_data.control_method = EFFORT;
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(EFFORT, mujoco_actuator_id));
                //mujoco_model_->actuator_gainprm[mujoco_actuator_id*mjNGAIN] = 1.0;
            }
            //mj_actuator_data.joint_name = joint_name;
            //mujoco_actuators_.insert(std::pair<std::string, MujocoActuatorData>(actuator_name, mj_actuator_data));
        }

        //for (auto& mujoco_actuator : mujoco_actuators_) {
        //    RCLCPP_DEBUG(this->nh_->get_logger(), "%s: %s", mujoco_actuator.first.c_str(), mujoco_actuator.second.to_string().c_str());
        //}

        auto string_to_double = [this](const std::string & input, double default_value=0.0) {
            if (!input.empty()) {
                double value = std::stod(input);
                RCLCPP_DEBUG(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
                return value;
            } else {
                return default_value;
            }
        };

        for (auto& joint_info : hardware_info.joints) {
            joints_.insert(std::pair<std::string, JointData>(joint_info.name, JointData()));
            JointData& joint = joints_.at(joint_info.name);
            joint.name = joint_info.name;
            joint.mujoco_joint_id = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint.name.c_str());
            joint.mujoco_qpos_addr = mujoco_model_->jnt_qposadr[joint.mujoco_joint_id];
            joint.mujoco_dofadr = mujoco_model_->jnt_dofadr[joint.mujoco_joint_id];

            // Get the limits from the urdf
            joint.upper_limit = joints.at(joint.name)->limits->upper;
            joint.lower_limit = joints.at(joint.name)->limits->lower;
            joint.velocity_limit = joints.at(joint.name)->limits->velocity;
            joint.effort_limit = joints.at(joint.name)->limits->effort;
            RCLCPP_DEBUG(rclcpp::get_logger("register joints"),
                        "%s: upper_limit: %f, lower_limit: %f, velocity_limit: %f, effort_limit: %f",
                        joint.name.c_str(), joint.upper_limit, joint.lower_limit, joint.velocity_limit, joint.effort_limit);
            if (joint.mujoco_joint_id == -1)
            {
                RCLCPP_WARN(this->nh_->get_logger(), "Joint %s not found in Mujoco model!", joint.name.c_str());
            }

            // Setup State Interfaces
            for(auto& state_interface : joint_info.state_interfaces) {
                if (state_interface.name == "position") {
                    joint.position = string_to_double(state_interface.initial_value);
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position));
                } else if (state_interface.name == "velocity") {
                    joint.velocity = string_to_double(state_interface.initial_value);
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity));
                } else if (state_interface.name == "effort") {
                    joint.effort = string_to_double(state_interface.initial_value);
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort));
                }
            }
            // Setup Command Interfaces
            for(auto& command_interface : joint_info.command_interfaces) {
                if (command_interface.name == "position") {
                    if (joint.position == 0.0) {
                        joint.position = string_to_double(command_interface.initial_value);
                    }
                    // initial position was not shown in rviz (mujoco had other joint_states than the robot model)
                    //mujoco_model_->qpos0[joint.mujoco_joint_id] = joint.position;
                    //mujoco_data_->qpos[joint.mujoco_qpos_addr] = joint.position;
                    //mj_forward(mujoco_model_, mujoco_data_);
                    joint.position_command = joint.position;
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position_command));
                    joint.control_method = POSITION;
                } else if (command_interface.name == "velocity") {
                    if (joint.velocity == 0.0) {
                        joint.velocity = string_to_double(command_interface.initial_value);
                    }
                    if (joint.velocity_limit == 0.0)  {
                        joint.velocity_limit = string_to_double(command_interface.max, 2.0);
                    }
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity_command));
                    joint.control_method = VELOCITY;
                } else if (command_interface.name == "effort") {
                    joint.effort = string_to_double(command_interface.initial_value);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort_command));
                    joint.control_method = EFFORT;
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
                    joint.second.control_method = NONE;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_stop_position");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    joint.second.control_method = NONE;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_stop_velocity");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    joint.second.control_method = NONE;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_stop_effort");
                }
            }
            for (const std::string &interface_name : start_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION) {
                    joint.second.control_method = POSITION;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_start_position for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    joint.second.control_method = VELOCITY;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_start_velocity for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    joint.second.control_method = EFFORT;
                    RCLCPP_DEBUG(nh_->get_logger(), "command_mode_start_effort for: %s", interface_name.c_str());
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        // read the joint states
        for (auto& joint_item : joints_) {
            JointData& joint = joint_item.second;
            joint.effort = mujoco_data_->qfrc_applied[joint.mujoco_dofadr];
            joint.velocity = mujoco_data_->qvel[joint.mujoco_dofadr];
            if (joint.type == urdf::Joint::PRISMATIC) {
                joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
            } else {
                joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
                //joint.position += angles::shortest_angular_distance(joint.position, mujoco_data_->qpos[joint.mujoco_qpos_addr]);
            }
        }
        return hardware_interface::return_type::OK;
    }

    //TODO: Add PID control and fix control with actuators
    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto &joint_data: joints_) {
            auto &joint = joint_data.second;
            if (joint.control_method == POSITION) {
                if (joint.actuators.find(POSITION) != joint.actuators.end()) {
                    mujoco_data_->ctrl[joint.actuators[POSITION]] = std::clamp(joint.position_command,
                                                                               joint.lower_limit,
                                                                               joint.upper_limit);
                } else {
                    mujoco_data_->qpos[joint.mujoco_qpos_addr] = std::clamp(joint.position_command,
                                                                            joint.lower_limit,
                                                                            joint.upper_limit);
                    mujoco_data_->qvel[joint.mujoco_dofadr] = std::clamp(joint.velocity,
                                                                         -joint.velocity_limit,
                                                                         joint.velocity_limit);
                }
            } else if (joint.control_method == VELOCITY) {
                if (joint.actuators.find(VELOCITY) != joint.actuators.end()) {
                    mujoco_data_->ctrl[joint.actuators[VELOCITY]] = std::clamp(joint.velocity,
                                                                               -joint.velocity_limit,
                                                                               joint.velocity_limit);
                } else {
                    mujoco_data_->qvel[joint.mujoco_dofadr] = std::clamp(joint.velocity,
                                                                         -joint.velocity_limit,
                                                                         joint.velocity_limit);
                }
            } else if (joint.control_method == EFFORT) {
                if (joint.actuators.find(EFFORT) != joint.actuators.end()) {
                    mujoco_data_->ctrl[joint.actuators[EFFORT]] = std::clamp(joint.effort,
                                                                             -joint.effort_limit,
                                                                             joint.effort_limit);
                } else {
                    mujoco_data_->qfrc_applied[joint.mujoco_dofadr] = std::clamp(joint.effort,
                                                                                 -joint.effort_limit,
                                                                                 joint.effort_limit);
                }
            }

            mj_forward(mujoco_model_, mujoco_data_);
        }

        return hardware_interface::return_type::OK;
    }

    bool MujocoSystem::string_ends_with(const std::string &value, const std::string &ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }

    bool MujocoSystem::extractSubstr(const std::string& str, const std::string& ending, std::string& joint_name) {
        size_t pos = str.rfind(ending);  // Find the position of the last occurrence of the ending substring
        if (pos == std::string::npos) {  // If the ending substring is not found
            return false;  // Return the original string
        }
        else {  // If the ending substring is found
            joint_name = str.substr(0, pos);  // Return the substring up to the ending substring
            return true;
        }
    }
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(
        mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)
