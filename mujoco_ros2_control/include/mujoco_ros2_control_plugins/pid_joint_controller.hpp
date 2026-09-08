/**
 * @file pid_joint_controller.hpp
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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__PID_JOINT_CONTROLLER_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__PID_JOINT_CONTROLLER_HPP_

#include <set>
#include <string>
#include <vector>

#include "mujoco_ros2_control/mujoco_ros2_control_plugin_interface.hpp"

namespace mujoco_ros2_control_plugins {

/**
 * @brief Claims a `<joint>`'s position/velocity/acceleration command
 *        interfaces and drives them into `qfrc_applied` with a software
 *        PD/PID loop -- the plugin equivalent of the built-in "no actuator"
 *        fallback MujocoSystem used to compute inline.
 *
 * Never claims "effort": that stays a built-in-only passthrough, same as
 * before. Whichever of "position"/"velocity"/"acceleration" the `<joint>`
 * actually declares gets claimed; the others (a real actuator, mimic wiring)
 * are untouched and keep going through MujocoSystem's built-in path.
 *
 * @code{.xml}
 * <joint name="left_hip_pitch_joint">
 *   <command_interface name="position"/>
 *   <param name="plugin">mujoco_ros2_control/PidJointController</param>
 *   <param name="kp">500</param>
 *   <param name="kd">2</param>
 * </joint>
 * @endcode
 */
class PidJointController : public mujoco_ros2_control::MujocoRos2ControlPluginInterface {
public:
    bool registerComponent(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::ComponentInfo &joint_info,
            const mujoco_ros2_control::JointLimits &joint_limits,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    std::set<std::string> claimed_command_interfaces() const override { return claimed_; }

    void read(const mjData *mujoco_data) override;
    void write(mjData *mujoco_data, double dt) override;

private:
    std::set<std::string> claimed_;

    mujoco_ros2_control::JointLimits limits_;
    int mujoco_qpos_addr_{-1};
    int mujoco_dofadr_{-1};

    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double kvff_{0.0};
    double kaff_{0.0};

    bool has_position_{false};
    bool has_velocity_{false};
    bool has_acceleration_{false};
    double position_command_{0.0};
    double velocity_command_{0.0};
    double acceleration_command_{0.0};

    // Persistent PID state, matching the deleted inline logic's semantics
    // exactly: one shared "last command" and integral/derivative state, since
    // position and velocity control were never simultaneously both run through
    // their own independent PID loop (velocity is a feedforward add-on when
    // position is also active -- see write()).
    double last_command_{0.0};
    double integral_{0.0};
    double prev_error_{0.0};

    rclcpp::Logger logger_ = rclcpp::get_logger("pid_joint_controller");
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PID_JOINT_CONTROLLER_HPP_
