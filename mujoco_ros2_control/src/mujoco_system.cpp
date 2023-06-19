/**
 * @file mujoco_visualization.cpp
 * @brief This file contains the implementation of the MujocoSystem class.
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
 *  derivative works or modifications of the software be licensed under the GPL-3.0 as well.
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *
 * This code is a modified version of the original code from Open Source Robotics Foundation, Inc.
 * https://github.com/ros-controls/gazebo_ros2_control/blob/master/gazebo_ros2_control/src/gazebo_system.cpp
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

#include "mujoco_ros2_control/mujoco_system.hpp"

namespace mujoco_ros2_control {

    bool MujocoSystem::initSim(
            mjModel *mujoco_model, mjData *mujoco_data,
            const hardware_interface::HardwareInfo &hardware_info,
            const urdf::Model *const urdf_model_ptr) {

        this->mujoco_model_ = mujoco_model;
        this->mujoco_data_ = mujoco_data;

        registerJoints(hardware_info, urdf_model_ptr->joints_);
        return true;
    }

    void MujocoSystem::registerJoints(const hardware_interface::HardwareInfo &hardware_info, const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints) {
        auto string_to_double = [this](const std::string & input, double default_value=0.0) {
            if (!input.empty()) {
                double value = std::stod(input);
                RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "\t\t\t found initial value: %f", value);
                return value;
            } else {
                return default_value;
            }
        };

        for (auto& joint_info : hardware_info.joints) {
            joints_.insert(std::pair<std::string, JointData>(joint_info.name, JointData()));
            // Create struct for joint with joint related datas
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
                RCLCPP_WARN(rclcpp::get_logger("mujoco_system"), "Joint %s not found in Mujoco model!", joint.name.c_str());
            }

            // Setup State Interfaces
            for(auto& state_interface : joint_info.state_interfaces) {
                if (state_interface.name == "position") {
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position));
                    joint.position = string_to_double(state_interface.initial_value);
                } else if (state_interface.name == "velocity") {
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity));
                    joint.velocity = string_to_double(state_interface.initial_value);
                } else if (state_interface.name == "effort") {
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort));
                    joint.effort = string_to_double(state_interface.initial_value);
                }
            }
            // Setup Command Interfaces
            for(auto& command_interface : joint_info.command_interfaces) {
                if (command_interface.name == "position") {
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_POSITION,
                            &joint.position_command));
                    joint.position_command = string_to_double(command_interface.initial_value);
                    // bring the joints in the initial position
                    if (joint.position_command == 0.0) {
                        joint.position_command = joint.position;
                    }
                    joint.control_methods.push_back(POSITION);
                } else if (command_interface.name == "velocity") {
                    if (joint.velocity_limit == 0.0)  {
                        joint.velocity_limit = string_to_double(command_interface.max, 2.0);
                    }
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity_command));
                    joint.velocity_command = string_to_double(command_interface.initial_value);
                } else if (command_interface.name == "effort") {
                    joint.effort = string_to_double(command_interface.initial_value);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort_command));
                    joint.effort_command = string_to_double(command_interface.initial_value);
                }
            }
            mj_forward(mujoco_model_, mujoco_data_);
        }

        for (int mujoco_actuator_id = 0; mujoco_actuator_id < mujoco_model_->nu; mujoco_actuator_id++) {
            std::string joint_name = mj_id2name(mujoco_model_, mjOBJ_JOINT, mujoco_model_->actuator_trnid[mujoco_actuator_id*2]);
            if (joints_[joint_name].name.empty()) {
                continue;
            }
            std::string actuator_name = mj_id2name(mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);

            double *dynprm = &mujoco_model_->actuator_dynprm[mujoco_actuator_id * mjNDYN];
            double *gainprm = &mujoco_model_->actuator_gainprm[mujoco_actuator_id * mjNGAIN];
            double *biasprm = &mujoco_model_->actuator_biasprm[mujoco_actuator_id * mjNBIAS];

            if (dynprm[0]  == 1 && dynprm[1]  == 0 && dynprm[2]  == 0 &&
                gainprm[0] == 1 && gainprm[1] == 0 && gainprm[2] == 0 &&
                biasprm[0] == 0 && biasprm[1] == 0 && biasprm[2] == 0) {
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(EFFORT, mujoco_actuator_id));
                RCLCPP_INFO(rclcpp::get_logger(actuator_name), "added effort actuator for joint: %s", joints_[joint_name].name.c_str());
            } else if(dynprm[0]  == 1 && dynprm[1] == 0 && dynprm[2]  == 0 &&
                      gainprm[0] == -1*biasprm[1] && gainprm[1] == 0 && gainprm[2] == 0 &&
                      biasprm[0] == 0 && biasprm[2] == 0) {
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(POSITION, mujoco_actuator_id));
                RCLCPP_INFO(rclcpp::get_logger(actuator_name), "added position actuator for joint: %s", joints_[joint_name].name.c_str());
            } else if(dynprm[0]  == 1 && dynprm[1] == 0 && dynprm[2]  == 0 &&
                      gainprm[0] == -1*biasprm[2] && gainprm[1] == 0 && gainprm[2] == 0 &&
                      biasprm[0] == 0 && biasprm[1] == 0) {
                joints_[joint_name].actuators.insert(std::pair<ControlMethod, int>(VELOCITY, mujoco_actuator_id));
                RCLCPP_INFO(rclcpp::get_logger(actuator_name), "added velocity actuator for joint: %s", joints_[joint_name].name.c_str());
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
            std::vector<ControlMethod> & control_methods = joint.second.control_methods;
            for (const std::string &interface_name : stop_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION) {
                    control_methods.erase(std::find(control_methods.begin(), control_methods.end(), POSITION));
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_position");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    control_methods.erase(std::find(control_methods.begin(), control_methods.end(), VELOCITY));
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_velocity");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    control_methods.erase(std::find(control_methods.begin(), control_methods.end(), EFFORT));
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_effort");
                }
            }
            for (const std::string &interface_name : start_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION) {
                    control_methods.push_back(POSITION);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_position for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY) {
                    control_methods.push_back(VELOCITY);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_velocity for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT) {
                    control_methods.clear();
                    control_methods.push_back(EFFORT);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_effort for: %s", interface_name.c_str());
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
            joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto &joint_data: joints_) {
            auto &joint = joint_data.second;
            auto & actuators = joint.actuators;
            auto & control_methods = joint.control_methods;
            // write position command to mujoco
            if (std::find(control_methods.begin(), control_methods.end(), POSITION) != control_methods.end()) {
                // get position command inside the limits
                double position = std::clamp(joint.position_command,
                                             joint.lower_limit,
                                             joint.upper_limit);
                // check if an actuator is available
                if (actuators.find(POSITION) != actuators.end()) {
                    // write to actuator ctrl
                    mujoco_data_->ctrl[actuators[POSITION]] = position;
                    mujoco_data_->qvel[joint.mujoco_dofadr] = joint.velocity_limit;
                } else {
                    // write to position and velocity address from the joint
                    mujoco_data_->qpos[joint.mujoco_qpos_addr] = position;
                    mujoco_data_->qvel[joint.mujoco_dofadr] = joint.velocity_limit;
                }
            }
            // write velocity command to mujoco
            if (std::find(control_methods.begin(), control_methods.end(), VELOCITY) != control_methods.end()) {
                // get velocity command inside the limits
                double velocity = std::clamp(joint.velocity_command,
                                             -joint.velocity_limit,
                                             joint.velocity_limit);
                // check if an actuator is available
                if (actuators.find(VELOCITY) != actuators.end()) {
                    // write to actuator ctrl
                    mujoco_data_->ctrl[actuators[VELOCITY]] = velocity;
                } else {
                    // write to velocity address from the joint
                    mujoco_data_->qvel[joint.mujoco_dofadr] = velocity;
                }
            }
            // write effort command to mujoco
            if (std::find(control_methods.begin(), control_methods.end(), EFFORT) != control_methods.end()) {
                // get effort command inside the limits
                double effort = std::clamp(joint.effort_command,
                                           -joint.effort_limit,
                                           joint.effort_limit);
                // check if an actuator is available
                if (actuators.find(EFFORT) != actuators.end()) {
                    mujoco_data_->ctrl[actuators[EFFORT]] = effort;
                } else {
                    // write to effort address from the joint
                    mujoco_data_->qfrc_applied[joint.mujoco_dofadr] = effort;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(
        mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)
