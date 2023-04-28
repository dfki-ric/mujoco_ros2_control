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
        this->n_dof_ = mujoco_model_->njnt - objects_in_scene;
        this->urdf_model_ = urdf_model;
        this->mujoco_model_ = mujoco_model;
        this->mujoco_data_ = mujoco_data;
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
        this->n_dof_ = hardware_info.joints.size();

        this->joint_names_.resize(this->n_dof_);
        this->joint_control_methods_.resize(this->n_dof_);
        this->joint_position_.resize(this->n_dof_);
        this->joint_velocity_.resize(this->n_dof_);
        this->joint_effort_.resize(this->n_dof_);
        this->joint_position_cmd_.resize(this->n_dof_);
        this->joint_velocity_cmd_.resize(this->n_dof_);
        this->joint_effort_cmd_.resize(this->n_dof_);
        this->joint_lower_limit_.resize(this->n_dof_);
        this->joint_upper_limit_.resize(this->n_dof_);
        this->joint_effort_limit_.resize(this->n_dof_);
        for (size_t j = 0; j < this->n_dof_; j++) {
            auto &joint_info = hardware_info.joints[j];
            std::string joint_name = this->joint_names_[j] = joint_info.name;

            MujocoJointData mj_joint_data{};
            mj_joint_data.id = mj_name2id(this->mujoco_model_, mjOBJ_JOINT, joint_info.name.c_str());
            mj_joint_data.qpos_addr = mujoco_model_->jnt_qposadr[mj_joint_data.id];
            mj_joint_data.qvel_addr = mujoco_model_->jnt_dofadr[mj_joint_data.id];
            mj_joint_data.type = mujoco_model_->jnt_type[mj_joint_data.id];
            this->sim_joints_.push_back(mj_joint_data);


            auto get_initial_value = [this](const hardware_interface::InterfaceInfo &interface_info) {
                if (!interface_info.initial_value.empty()) {
                    double value = std::stod(interface_info.initial_value);
                    RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
                    return value;
                } else {
                    return 0.0;
                }
            };

            // register the state handles
            double initial_position = std::numeric_limits<double>::quiet_NaN();
            double initial_velocity = std::numeric_limits<double>::quiet_NaN();
            double initial_effort = std::numeric_limits<double>::quiet_NaN();
            //double lower_limit = std::numeric_limits<double>::quiet_NaN();
            //double upper_limit = std::numeric_limits<double>::quiet_NaN();
            //double effort_limit = std::numeric_limits<double>::quiet_NaN();

            for (size_t i = 0; i < joint_info.state_interfaces.size(); i++) {
                if (joint_info.state_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_POSITION,
                            &this->joint_position_[j]);
                    initial_position = std::stod(joint_info.state_interfaces[i].initial_value);
                    //lower_limit = std::stod(joint_info.state_interfaces[i].min);
                    //upper_limit = std::stod(joint_info.state_interfaces[i].max);
                    this->joint_position_[j] = initial_position;
                    //this->joint_lower_limit_[j] = lower_limit;
                    //this->joint_lower_limit_[j] = upper_limit;

                } else if (joint_info.state_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &this->joint_velocity_[j]);
                    initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
                    this->joint_velocity_[j] = initial_velocity;
                } else if (joint_info.state_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_EFFORT,
                            &this->joint_effort_[j]);
                    initial_effort = get_initial_value(joint_info.state_interfaces[i]);
                    //effort_limit = std::stod(joint_info.state_interfaces[i].max);
                    this->joint_effort_[j] = initial_effort;
                    //this->joint_effort_limit_[j] = effort_limit;
                }
            }
            // Register the command handles
            for (size_t i = 0; i < joint_info.command_interfaces.size(); i++) {
                if (joint_info.command_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_POSITION,
                            &this->joint_position_cmd_[j]);
                    if (!std::isnan(initial_position)) {
                        this->joint_position_cmd_[j] = initial_position;
                    }
                } else if (joint_info.command_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &this->joint_velocity_cmd_[j]);
                    if (!std::isnan(initial_position)) {
                        this->joint_velocity_cmd_[j] = initial_velocity;
                    }
                } else if (joint_info.command_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_EFFORT,
                            &this->joint_effort_cmd_[j]);
                    if (!std::isnan(initial_effort)) {
                        this->joint_effort_cmd_[j] = initial_effort;
                    }
                }
                if (!std::isnan(initial_position)) {
                    this->joint_position_cmd_[j] = initial_position;
                    //this->joint_lower_limit_[j] = lower_limit;
                    //this->joint_lower_limit_[j] = upper_limit;
                }
                if (!std::isnan(initial_velocity)) {
                    this->joint_velocity_cmd_[j] = initial_velocity;
                }
                if (!std::isnan(initial_position)) {
                    this->joint_effort_cmd_[j] = initial_effort;
                    //this->joint_effort_limit_[j] = effort_limit;
                }
            }
        }

        for (int mujoco_actuator_id = 0; mujoco_actuator_id < this->mujoco_model_->nu; mujoco_actuator_id++) {
            std::string actuator_name = mj_id2name(this->mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);
            MujocoActuatorData mj_actuator_data{};
            mj_actuator_data.id = mujoco_actuator_id;
            mujoco_actuators_.insert(std::pair<std::string, MujocoActuatorData>(actuator_name, mj_actuator_data));
        }
        for (auto &mujoco_actuator: mujoco_actuators_) {
            RCLCPP_DEBUG(this->nh_->get_logger(), "%s: %s", mujoco_actuator.first.c_str(),
                         mujoco_actuator.second.to_string().c_str());
        }

        for (std::string name: this->joint_names_) {

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

        for (size_t j = 0; j < this->joint_names_.size(); j++) {
            for (const std::string &interface_name: stop_interfaces) {
                // Clear joint control method bits corresponding to stop interfaces
                if (interface_name == (this->joint_names_[j] + "/" +
                                       hardware_interface::HW_IF_POSITION)) {
                    this->joint_control_methods_[j] &= static_cast<ControlMethod_>(VELOCITY & EFFORT);
                } else if (interface_name == (this->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_VELOCITY)) {
                    this->joint_control_methods_[j] &= static_cast<ControlMethod_>(POSITION & EFFORT);
                } else if (interface_name == (this->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_EFFORT)) {
                    this->joint_control_methods_[j] &=
                            static_cast<ControlMethod_>(POSITION & VELOCITY);
                }
            }

            // Set joint control method bits corresponding to start interfaces
            for (const std::string &interface_name: start_interfaces) {
                if (interface_name == (this->joint_names_[j] + "/" +
                                       hardware_interface::HW_IF_POSITION)) {
                    this->joint_control_methods_[j] |= POSITION;
                } else if (interface_name == (this->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_VELOCITY)) {
                    this->joint_control_methods_[j] |= VELOCITY;
                } else if (interface_name == (this->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_EFFORT)) {
                    this->joint_control_methods_[j] |= EFFORT;
                }
            }
        }

        // mimic joint has the same control mode as mimicked joint
        for (const auto &mimic_joint: this->mimic_joints_) {
            this->joint_control_methods_[mimic_joint.joint_index] =
                    this->joint_control_methods_[mimic_joint.mimicked_joint_index];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (size_t j = 0; j < this->joint_names_.size(); j++) {
            if (string_ends_with(this->joint_names_[j], "FJ1") ||
                string_ends_with(this->joint_names_[j], "FJ2")) {
                std::string actuator_name = this->joint_names_[j];
                actuator_name[actuator_name.size() - 1] = '0';
                MujocoActuatorData &actuator = mujoco_actuators_.at(actuator_name);
                this->joint_effort_[j] = mujoco_data_->qfrc_actuator[actuator.id] / 2;
            } else {
                this->joint_effort_[j] = mujoco_data_->qfrc_applied[this->sim_joints_[j].qpos_addr];
            }
            if (this->sim_joints_[j].type == urdf::Joint::PRISMATIC) {
                this->joint_position_[j] = mujoco_data_->qpos[this->sim_joints_[j].qpos_addr];
            } else {
                this->joint_position_[j] += angles::shortest_angular_distance(
                        this->joint_position_[j], mujoco_data_->qpos[this->sim_joints_[j].qpos_addr]);
            }
            this->joint_velocity_[j] = mujoco_data_->qvel[this->sim_joints_[j].qvel_addr];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto &actuator: mujoco_actuators_) {
            if (string_ends_with(actuator.first, "FJ0")) {
                std::string joint_1_name = actuator.first;
                std::string joint_2_name = actuator.first;
                joint_1_name[joint_1_name.size() - 1] = '1';
                joint_2_name[joint_2_name.size() - 1] = '2';
                size_t index_joint_1;
                size_t index_joint_2;

                auto it = find(this->joint_names_.begin(), this->joint_names_.end(), joint_1_name);
                if (it != this->joint_names_.end()) {
                    index_joint_1 = it - this->joint_names_.begin();
                } else {
                    continue;
                }
                it = find(this->joint_names_.begin(), this->joint_names_.end(), joint_2_name);
                if (it != this->joint_names_.end()) {
                    index_joint_2 = it - this->joint_names_.begin();
                } else {
                    continue;
                }
                if (this->joint_control_methods_[index_joint_1] & EFFORT) {
                    mujoco_data_->ctrl[actuator.second.id] = this->joint_effort_cmd_[index_joint_1] +
                                                             this->joint_effort_cmd_[index_joint_2];
                }

                if (this->joint_control_methods_[index_joint_1] & POSITION) {
                    mujoco_data_->ctrl[actuator.second.id] = this->joint_position_cmd_[index_joint_1] +
                                                             this->joint_position_cmd_[index_joint_2];
                }

                if (this->joint_control_methods_[index_joint_1] & VELOCITY) {
                    mujoco_data_->ctrl[actuator.second.id] = this->joint_velocity_cmd_[index_joint_1] +
                                                             this->joint_velocity_cmd_[index_joint_1];
                }
            }
            for (size_t j = 0; j < this->joint_names_.size(); j++) {
                if (actuator.first == this->joint_names_[j]) {
                    if (this->joint_control_methods_[j] & EFFORT) {
                        mujoco_data_->ctrl[actuator.second.id] = this->joint_effort_cmd_[j];
                    }
                    if (this->joint_control_methods_[j] & POSITION) {
                        mujoco_data_->ctrl[actuator.second.id] = this->joint_position_cmd_[j];
                    }
                    if (this->joint_control_methods_[j] & VELOCITY) {
                        mujoco_data_->ctrl[actuator.second.id] = this->joint_velocity_cmd_[j];
                    }
                }
            }
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
