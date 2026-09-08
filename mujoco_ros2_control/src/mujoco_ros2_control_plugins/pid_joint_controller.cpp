/**
 * @file pid_joint_controller.cpp
 * @brief Software PD/PID joint controller: drives a position/velocity/
 *        acceleration command that has no matching MuJoCo actuator.
 *
 * @author Adrian Danzglock
 * @date 2026
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2026, DFKI GmbH
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
 */

#include "mujoco_ros2_control_plugins/pid_joint_controller.hpp"

#include <algorithm>
#include <initializer_list>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mujoco_ros2_control_plugins {

namespace {

double string_to_double(const std::string &input, double default_value = 0.0) {
    return input.empty() ? default_value : std::stod(input);
}

/// First matching `<param>` among @p keys, or 0.0 if none is set.
double gain_param(const hardware_interface::ComponentInfo &joint_info,
                   std::initializer_list<const char *> keys) {
    for (const char *key : keys) {
        const auto it = joint_info.parameters.find(key);
        if (it != joint_info.parameters.end()) {
            return string_to_double(it->second);
        }
    }
    return 0.0;
}

/// A named interface's `initial_value` `<param>`, or 0.0 if the interface (or
/// the param) is absent -- used to seed a claimed command from whichever
/// initial value the <joint> declared for it.
double initial_value(const std::vector<hardware_interface::InterfaceInfo> &interfaces,
                      const std::string &name) {
    for (const auto &interface_info : interfaces) {
        if (interface_info.name == name) {
            return string_to_double(interface_info.initial_value);
        }
    }
    return 0.0;
}

}  // namespace

bool PidJointController::registerComponent(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::ComponentInfo &joint_info,
        const mujoco_ros2_control::JointLimits &joint_limits,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        std::vector<hardware_interface::CommandInterface> &command_interfaces) {
    (void)state_interfaces;
    logger_ = node->get_logger().get_child(name_);

    const int mujoco_joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, joint_info.name.c_str());
    if (mujoco_joint_id < 0) {
        RCLCPP_ERROR(logger_, "Joint '%s' not found in the MuJoCo model.", joint_info.name.c_str());
        return false;
    }
    mujoco_qpos_addr_ = mujoco_model->jnt_qposadr[mujoco_joint_id];
    mujoco_dofadr_ = mujoco_model->jnt_dofadr[mujoco_joint_id];

    limits_ = joint_limits;

    kp_ = gain_param(joint_info, {"p", "kp"});
    ki_ = gain_param(joint_info, {"i", "ki"});
    kd_ = gain_param(joint_info, {"d", "kd"});
    kaff_ = gain_param(joint_info, {"aff", "kaff"});
    kvff_ = gain_param(joint_info, {"vff", "kvff"});

    for (const auto &command_interface : joint_info.command_interfaces) {
        if (command_interface.name == "position") {
            has_position_ = true;
            claimed_.insert("position");
            command_interfaces.emplace_back(joint_info.name, hardware_interface::HW_IF_POSITION, &position_command_);
            position_command_ = string_to_double(command_interface.initial_value);
            if (position_command_ == 0.0) {
                // Bring the joint to its initial pose, same fallback the
                // built-in path applies for an actuator-backed position joint.
                position_command_ = initial_value(joint_info.state_interfaces, "position");
            }
        } else if (command_interface.name == "velocity") {
            has_velocity_ = true;
            claimed_.insert("velocity");
            command_interfaces.emplace_back(joint_info.name, hardware_interface::HW_IF_VELOCITY, &velocity_command_);
            velocity_command_ = string_to_double(command_interface.initial_value);
            if (limits_.velocity == 0.0) {
                limits_.velocity = string_to_double(command_interface.max, 2.0);
            }
        } else if (command_interface.name == "acceleration") {
            has_acceleration_ = true;
            claimed_.insert("acceleration");
            command_interfaces.emplace_back(joint_info.name, hardware_interface::HW_IF_ACCELERATION, &acceleration_command_);
            acceleration_command_ = string_to_double(command_interface.initial_value);
        }
    }

    if (claimed_.empty()) {
        RCLCPP_ERROR(logger_,
            "Joint '%s' names this plugin but declares none of position/velocity/"
            "acceleration -- nothing for it to claim.", joint_info.name.c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "Joint '%s': PID claimed position=%s velocity=%s acceleration=%s "
                "(kp=%f ki=%f kd=%f kvff=%f kaff=%f)",
                joint_info.name.c_str(),
                has_position_ ? "yes" : "no", has_velocity_ ? "yes" : "no",
                has_acceleration_ ? "yes" : "no", kp_, ki_, kd_, kvff_, kaff_);
    return true;
}

void PidJointController::read(const mjData *mujoco_data) {
    (void)mujoco_data;
}

void PidJointController::write(mjData *mujoco_data, double dt) {
    double tau = 0.0;

    if (has_position_) {
        double position = std::clamp(position_command_, limits_.lower, limits_.upper);
        double position_error = position - mujoco_data->qpos[mujoco_qpos_addr_];
        if (last_command_ != position) {
            last_command_ = position;
            integral_ = position_error;
        } else {
            integral_ += position_error;
        }
        double derivative = (position_error - prev_error_) / dt;
        tau = kp_ * position_error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = position_error;
    }

    if (has_velocity_) {
        double velocity = std::clamp(velocity_command_, -limits_.velocity, limits_.velocity);
        if (has_position_) {
            // Feedforward only: position's PID loop above already owns tau.
            tau += kvff_ * velocity;
        } else {
            double velocity_error = velocity - mujoco_data->qvel[mujoco_dofadr_];
            if (last_command_ != velocity) {
                last_command_ = velocity;
                integral_ = velocity_error;
            } else {
                integral_ += velocity_error;
            }
            double derivative = (velocity_error - prev_error_) / dt;
            tau = kp_ * velocity_error + ki_ * integral_ + kd_ * derivative;
            prev_error_ = velocity_error;
        }
    }

    // Only meaningful as a feedforward atop combined position+velocity PID,
    // same as the built-in path this replaces -- acceleration alone still
    // drives nothing.
    if (has_position_ && has_velocity_ && has_acceleration_) {
        double acceleration = std::clamp(acceleration_command_, -limits_.acceleration, limits_.acceleration);
        tau += kaff_ * acceleration;
    }

    if (has_position_ || has_velocity_) {
        double tau_cmd = std::clamp(tau, -limits_.effort, limits_.effort);
        mujoco_data->qfrc_applied[mujoco_dofadr_] = tau_cmd;
    }
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_control_plugins::PidJointController, mujoco_ros2_control::MujocoRos2ControlPluginInterface)
