/**
 * @file mujoco_system.cpp
 * @brief This file contains the implementation of the MujocoSystem class.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <mujoco_ros2_control/mujoco_system.hpp>

namespace mujoco_ros2_control {

    bool MujocoSystem::initSim(
            mjModel *mujoco_model, mjData *mujoco_data,
            const hardware_interface::HardwareInfo &hardware_info,
            const urdf::Model *const urdf_model_ptr) {

        this->mujoco_model_ = mujoco_model;
        this->mujoco_data_ = mujoco_data;

#if defined(ROS_DISTRO_HUMBLE)
        // Humble's SystemInterface has no get_logger() accessor -- it is only
        // populated from HardwareComponentParams.logger, passed to on_init() on
        // Jazzy+ -- so this file's own component-named logger stands in for it.
        const rclcpp::Logger logger = rclcpp::get_logger("mujoco_system");
#else
        const rclcpp::Logger logger = get_logger();
#endif
        const std::map<std::string, JointLimits> joint_limits =
            resolveJointLimits(hardware_info, urdf_model_ptr->joints_);

        std::map<std::string, ClaimedInterfaces> claimed_interfaces;
        if (sim_node_) {
            plugins_.registerSensors(
                sim_node_, mujoco_model, hardware_info, state_interfaces_, command_interfaces_, logger);
            plugins_.registerGpios(
                sim_node_, mujoco_model, hardware_info, state_interfaces_, command_interfaces_, logger);
            claimed_interfaces = plugins_.registerJoints(
                sim_node_, mujoco_model, hardware_info, joint_limits, state_interfaces_, command_interfaces_, logger);
        } else {
            RCLCPP_ERROR(logger,
                "No simulation node was provided, so <sensor>/<gpio>/<joint> plugins cannot "
                "be loaded. MujocoResourceManager::setSimNode() was not called.");
        }

        registerJoints(hardware_info, urdf_model_ptr->joints_, claimed_interfaces);

        std::vector<std::string> joints_to_remove = {};
        for (const auto &joint : joints_) {
            if (joint.first != joint.second.name) {
                joints_to_remove.push_back(joint.first);
            }
        }
        for (const auto &joint : joints_to_remove) {
            joints_.erase(joint);
        }
        return true;
    }

    std::map<std::string, JointLimits> MujocoSystem::resolveJointLimits(
            const hardware_interface::HardwareInfo &hardware_info,
            const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints) {
        std::map<std::string, JointLimits> limits;
        for (const auto &joint_info : hardware_info.joints) {
            const auto urdf_joint_it = joints.find(joint_info.name);
            if (urdf_joint_it == joints.end()) {
                continue;
            }
            const urdf::Joint &urdf_joint = *urdf_joint_it->second;

            JointLimits joint_limits;
            if (urdf_joint.type == urdf::Joint::REVOLUTE || urdf_joint.type == urdf::Joint::PRISMATIC) {
                joint_limits.upper = urdf_joint.limits->upper;
                joint_limits.lower = urdf_joint.limits->lower;
            }
            if (urdf_joint.type == urdf::Joint::REVOLUTE || urdf_joint.type == urdf::Joint::PRISMATIC ||
                urdf_joint.type == urdf::Joint::CONTINUOUS) {
                if (urdf_joint.limits != nullptr) {
                    joint_limits.velocity = urdf_joint.limits->velocity;
                    joint_limits.effort = urdf_joint.limits->effort;
                }
            }
            limits[joint_info.name] = joint_limits;
        }
        return limits;
    }

    void MujocoSystem::registerJoints(const hardware_interface::HardwareInfo &hardware_info,
                                       const std::map<std::string, std::shared_ptr<urdf::Joint>> &joints,
                                       const std::map<std::string, ClaimedInterfaces> &claimed_interfaces) {
        auto string_to_double = [this](const std::string & input, double default_value=0.0) {
            if (!input.empty()) {
                double value = std::stod(input);
                RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "\t\t\t found initial value: %f", value);
                return value;
            } else {
                return default_value;
            }
        };
        name_ = hardware_info.name;

        static const ClaimedInterfaces kNoClaims{};

        RCLCPP_INFO(rclcpp::get_logger(hardware_info.name.c_str()), "Initializing Hardware Interface");
        for (auto& joint_info : hardware_info.joints) {
            RCLCPP_INFO(rclcpp::get_logger(hardware_info.name.c_str()), "  %s", joint_info.name.c_str());
            const auto claims_it = claimed_interfaces.find(joint_info.name);
            const ClaimedInterfaces &claims =
                (claims_it != claimed_interfaces.end()) ? claims_it->second : kNoClaims;
            if (joints.find(joint_info.name) == joints.end()) {
                RCLCPP_WARN(rclcpp::get_logger("mujoco_system"),
                            "Joint %s was not found in the URDF, registration of joint failed", joint_info.name.c_str());
                continue;
            }
            const int mj_joint_id = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint_info.name.c_str());
            if (mj_joint_id == -1) {
                RCLCPP_WARN(rclcpp::get_logger("mujoco_system"), "Joint %s not found in Mujoco model!", joint_info.name.c_str());
                continue;
            }
            joints_.insert(std::pair<std::string, JointData>(joint_info.name, JointData()));
            // Create struct for joint with joint related datas
            JointData& joint = joints_.at(joint_info.name);
            joint.name = joint_info.name;
            joint.claimed_command_interfaces = claims.command;

            joint.mujoco_joint_id = mj_joint_id;
            joint.mujoco_qpos_addr = mujoco_model_->jnt_qposadr[joint.mujoco_joint_id];
            joint.mujoco_dofadr = mujoco_model_->jnt_dofadr[joint.mujoco_joint_id];

            joint.type = joints.at(joint.name)->type;

            // Get the limits from the urdf (except for continuous joints)
            if (joint.type == urdf::Joint::REVOLUTE || joint.type == urdf::Joint::PRISMATIC) {
                joint.upper_limit = joints.at(joint.name)->limits->upper;
                joint.lower_limit = joints.at(joint.name)->limits->lower;
            }

            // Read velocity and effort limits from the URDF
            if (joint.type == urdf::Joint::REVOLUTE || joint.type == urdf::Joint::PRISMATIC ||
                joint.type == urdf::Joint::CONTINUOUS) {
                if (joints.at(joint.name)->limits != nullptr) {
                    joint.velocity_limit = joints.at(joint.name)->limits->velocity;
                    joint.effort_limit = joints.at(joint.name)->limits->effort;
                }
            }

            if (joints.at(joint.name)->mimic != nullptr) {
                if (joints.find(joints.at(joint.name)->mimic->joint_name) == joints.end()) {
                    RCLCPP_WARN(rclcpp::get_logger("mujoco_system"),
                                "Mimiced Joint %s was not found in the URDF, registration of mimic failed", joint_info.name.c_str());
                } else if (joints.at(joints.at(joint.name)->mimic->joint_name)->type == joints.at(joint.name)->type) {
                    MimicJoint mj;
                    mj.joint = joint.name;
                    mj.mimiced_joint = joints.at(joint.name)->mimic->joint_name;
                    mj.mimic_multiplier = joints.at(joint.name)->mimic->multiplier;
                    mj.mimic_offset = joints.at(joint.name)->mimic->offset;
                    mimiced_joints_.push_back(mj);
                    RCLCPP_INFO(rclcpp::get_logger("mujoco_system"), "Joint %s mimic Joint %s", mj.joint.c_str(), mj.mimiced_joint.c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("mujoco_system"), "Mimic joints must be from the same joint type");
                }
            }

            RCLCPP_DEBUG(rclcpp::get_logger("register joints"),
                         "%s: jnt_limited: %hhu, jnt_range: [%f,%f], jnt_actfrclimited: %hhu, jnt_actfrcrange: [%f, %f]",
                         joint.name.c_str(),
                         mujoco_model_->jnt_limited[joint.mujoco_joint_id],
                         mujoco_model_->jnt_range[joint.mujoco_joint_id*2], mujoco_model_->jnt_range[joint.mujoco_joint_id*2+1],
                         mujoco_model_->jnt_actfrclimited[joint.mujoco_joint_id],
                         mujoco_model_->jnt_actfrcrange[joint.mujoco_joint_id*2], mujoco_model_->jnt_actfrcrange[joint.mujoco_joint_id*2+1]);

            RCLCPP_DEBUG(rclcpp::get_logger("register joints"),
                        "%s: upper_limit: %f, lower_limit: %f, velocity_limit: %f, effort_limit: %f",
                        joint.name.c_str(), joint.upper_limit, joint.lower_limit, joint.velocity_limit, joint.effort_limit);

            // Setup State Interfaces
            for(auto& state_interface : joint_info.state_interfaces) {
                if (claims.state.count(state_interface.name)) {
                    // Claimed by a plugin; it exported this interface itself.
                    continue;
                }
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
                } else if (state_interface.name == "acceleration") {
                    joint.state_interfaces.emplace_back(&state_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_ACCELERATION,
                            &joint.acceleration));
                    joint.acceleration = string_to_double(state_interface.initial_value);
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
                if (claims.command.count(command_interface.name)) {
                    // Claimed by a plugin; it exported this interface (and drives
                    // its control method) itself -- see MujocoRos2ControlPluginLoader.
                    continue;
                }
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
                    joint.control_methods.push_back(VELOCITY);
                } else if (command_interface.name == "acceleration") {
                    joint.acceleration = string_to_double(command_interface.initial_value);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_ACCELERATION,
                            &joint.acceleration_command));
                    joint.effort_command = string_to_double(command_interface.initial_value);
                    joint.control_methods.push_back(ACCELERATION);
                } else if (command_interface.name == "effort") {
                    joint.effort = string_to_double(command_interface.initial_value);
                    joint.command_interfaces.emplace_back(&command_interfaces_.emplace_back(
                            joint.name,
                            hardware_interface::HW_IF_EFFORT,
                            &joint.effort_command));
                    joint.effort_command = string_to_double(command_interface.initial_value);
                    joint.control_methods.push_back(EFFORT);
                }
            }

            if (joints.at(joint.name)->mimic != nullptr) {
                if (joints.find(joints.at(joint.name)->mimic->joint_name) == joints.end()) {
                    RCLCPP_WARN(rclcpp::get_logger("mujoco_system"),
                                "Mimiced Joint %s was not found in the URDF, registration of mimic failed", joint_info.name.c_str());
                    continue;
                }
                if (joints.at(joints.at(joint.name)->mimic->joint_name)->type == joints.at(joint.name)->type) {
                    MimicJoint mj;
                    mj.joint = joint.name;
                    mj.mimiced_joint = joints.at(joint.name)->mimic->joint_name;
                    mj.mimic_multiplier = joints.at(joint.name)->mimic->multiplier;
                    mj.mimic_offset = joints.at(joint.name)->mimic->offset;
                    mimiced_joints_.push_back(mj);
                    RCLCPP_INFO(rclcpp::get_logger("mujoco_system"), "Joint %s mimic Joint %s", mj.joint.c_str(), mj.mimiced_joint.c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("mujoco_system"), "Mimic joints must be from the same joint type");
                }
            }

            mj_forward(mujoco_model_, mujoco_data_);
        }
        

        for (int mujoco_actuator_id = 0; mujoco_actuator_id < mujoco_model_->nu; mujoco_actuator_id++) {
            const char *joint_name_c = mj_id2name(
                mujoco_model_, mjOBJ_JOINT,
                mujoco_model_->actuator_trnid[mujoco_actuator_id * 2]);
            if (!joint_name_c) {
                continue;
            }
            auto joint_it = joints_.find(joint_name_c);
            if (joint_it == joints_.end() || joint_it->second.name.empty()) {
                continue;
            }
            const std::string &joint_name = joint_it->first;
            std::string actuator_name;
            const char* act_name = mj_id2name(mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);
            if(act_name) {
                actuator_name = act_name;
            } else {
                actuator_name = "actuator" + std::to_string(mujoco_actuator_id);
            }

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

        std::vector<uint> indices;  // Indices from invalid mimiced joints
        for (size_t i = 0; i < mimiced_joints_.size(); i++) {
            const auto &mj = mimiced_joints_[i];
            if (joints_.find(mj.mimiced_joint) == joints_.end()) {
                indices.insert(indices.begin(), i);
                RCLCPP_WARN(rclcpp::get_logger("mujoco_system"),
                            "%s: Mimiced Joint %s was not found in the registered joints, registration of mimic failed", mj.joint.c_str(), mj.mimiced_joint.c_str());
            }
        }
        for(const auto& index: indices) {
            if (index < mimiced_joints_.size()) {
                mimiced_joints_.erase(mimiced_joints_.begin() + index);
            }
        }
    }

#if defined(ROS_DISTRO_HUMBLE)
    CallbackReturn
    MujocoSystem::on_init(const hardware_interface::HardwareInfo &system_info) {
        if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }
#else
    CallbackReturn
    MujocoSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }
#endif

    std::vector<hardware_interface::StateInterface>
    MujocoSystem::export_state_interfaces() {
        return std::move(this->state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface>
    MujocoSystem::export_command_interfaces() {
        return std::move(this->command_interfaces_);
    }

    CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
        plugins_.activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MujocoSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        plugins_.deactivate();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    MujocoSystem::perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) {
        for (auto& joint : joints_) {
            std::vector<ControlMethod> & control_methods = joint.second.control_methods;
            const auto &claimed = joint.second.claimed_command_interfaces;
            for (const std::string &interface_name : stop_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION &&
                    !claimed.count(hardware_interface::HW_IF_POSITION)) {
                    if (!control_methods.empty()) {
                        control_methods.erase(std::find(control_methods.begin(), control_methods.end(), POSITION));
                    }
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_position");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY &&
                    !claimed.count(hardware_interface::HW_IF_VELOCITY)) {
                    if (!control_methods.empty()) {
                        control_methods.erase(std::find(control_methods.begin(), control_methods.end(), VELOCITY));
                    }
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_velocity");
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT &&
                    !claimed.count(hardware_interface::HW_IF_EFFORT)) {
                    if (!control_methods.empty()) {
                        control_methods.erase(std::find(control_methods.begin(), control_methods.end(), EFFORT));
                    }
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_stop_effort");
                }
            }
            for (const std::string &interface_name : start_interfaces) {
                if (interface_name == joint.first + "/" + hardware_interface::HW_IF_POSITION &&
                    !claimed.count(hardware_interface::HW_IF_POSITION)) {
                    if (!control_methods.empty()) {
                        control_methods.erase(std::find(control_methods.begin(), control_methods.end(), POSITION));
                    }
                    control_methods.push_back(POSITION);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_position for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_VELOCITY &&
                    !claimed.count(hardware_interface::HW_IF_VELOCITY)) {
                    if (!control_methods.empty()) {
                        control_methods.erase(std::find(control_methods.begin(), control_methods.end(), VELOCITY));
                    }
                    control_methods.push_back(VELOCITY);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_velocity for: %s", interface_name.c_str());
                } else if (interface_name == joint.first + "/" + hardware_interface::HW_IF_EFFORT &&
                    !claimed.count(hardware_interface::HW_IF_EFFORT)) {
                    if (!control_methods.empty()) {
                        control_methods.clear();
                    }
                    control_methods.push_back(EFFORT);
                    RCLCPP_DEBUG(rclcpp::get_logger("mujoco_system"), "command_mode_start_effort for: %s", interface_name.c_str());
                }
            }
        }

        if (!plugins_.performCommandModeSwitch(start_interfaces, stop_interfaces)) {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        // read the joint states
        for (auto& joint_item : joints_) {
            JointData& joint = joint_item.second;
            joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
            joint.velocity = mujoco_data_->qvel[joint.mujoco_dofadr];
            joint.effort = mujoco_data_->qfrc_actuator[joint.mujoco_dofadr]
                         + mujoco_data_->qfrc_applied[joint.mujoco_dofadr];
        }

        plugins_.readAll(mujoco_data_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (const auto &mj : mimiced_joints_) {
            joints_.at(mj.joint).control_methods = joints_.at(mj.mimiced_joint).control_methods;
            joints_.at(mj.joint).position_command = joints_.at(mj.mimiced_joint).position_command * mj.mimic_multiplier + mj.mimic_offset;
            joints_.at(mj.joint).velocity_command = joints_.at(mj.mimiced_joint).velocity_command * mj.mimic_multiplier + mj.mimic_offset;
            joints_.at(mj.joint).effort_command = joints_.at(mj.mimiced_joint).effort_command * mj.mimic_multiplier + mj.mimic_offset;
        }

        for (auto &joint_data: joints_) {
            if (joint_data.first != joint_data.second.name) {
                continue;
            }
            auto &joint = joint_data.second;
            auto & actuators = joint.actuators;
            auto & control_methods = joint.control_methods;

            // Position Control
            if (std::find(control_methods.begin(), control_methods.end(), POSITION) != control_methods.end()) {
                // get position command inside the limits
                double position = std::clamp(joint.position_command,
                                             joint.lower_limit,
                                             joint.upper_limit);

                // check if an actuator is available
                if (actuators.find(POSITION) != actuators.end()) {
                    // write to actuator ctrl
                    if (position != joint.last_command) {
                        joint.last_command = position;
                        mujoco_data_->ctrl[actuators[POSITION]] = position;
                    }
                } else if (!joint.warned_no_position_control) {
                    // No actuator, and (since it would otherwise have been excluded
                    // from control_methods entirely, see registerJoints()) no plugin
                    // claiming "position" either -- nothing drives this command.
                    RCLCPP_ERROR(rclcpp::get_logger("mujoco_system"),
                        "Joint '%s' has an active position command with no matching "
                        "actuator and no <param name=\"plugin\"> claiming it; this "
                        "command is being ignored.", joint.name.c_str());
                    joint.warned_no_position_control = true;
                }
            }

            // Velocity Control
            if (std::find(control_methods.begin(), control_methods.end(), VELOCITY) != control_methods.end()) {
                // get velocity command inside the limits
                double velocity = std::clamp(joint.velocity_command,
                                             -joint.velocity_limit,
                                             joint.velocity_limit);
                // check if an actuator is available
                if (actuators.find(VELOCITY) != actuators.end()) {
                    // write to actuator ctrl
                    if (velocity != joint.last_command) {
                        mujoco_data_->ctrl[actuators[VELOCITY]] = velocity;
                    }
                } else if (!joint.warned_no_velocity_control) {
                    RCLCPP_ERROR(rclcpp::get_logger("mujoco_system"),
                        "Joint '%s' has an active velocity command with no matching "
                        "actuator and no <param name=\"plugin\"> claiming it; this "
                        "command is being ignored.", joint.name.c_str());
                    joint.warned_no_velocity_control = true;
                }
            }

            // Effort Control
            if (std::find(control_methods.begin(), control_methods.end(), EFFORT) != control_methods.end()) {
                // get effort command inside the limits
                double effort = std::clamp(joint.effort_command,
                                           -joint.effort_limit,
                                           joint.effort_limit);
                // check if an actuator is available
                if (actuators.find(EFFORT) != actuators.end()) {
                    mujoco_data_->ctrl[actuators[EFFORT]] = effort;
                    continue;
                } else {
                    // write to effort address from the joint
                    mujoco_data_->qfrc_applied[joint.mujoco_dofadr] = effort;
                }
            }
        }

        plugins_.writeAll(mujoco_data_, period.seconds());

        return hardware_interface::return_type::OK;
    }
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(
        mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)
