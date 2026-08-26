/**
 * @file unitree_hg_low_level.hpp
 * @brief Plugin serving the unitree_hg LowCmd/LowState interface from inside MuJoCo.
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

#ifndef MUJOCO_ROS2_CONTROL_EXAMPLES__UNITREE_HG_LOW_LEVEL_HPP_
#define MUJOCO_ROS2_CONTROL_EXAMPLES__UNITREE_HG_LOW_LEVEL_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"

namespace mujoco_ros2_control_examples {

/**
 * @brief The Unitree G1/H1-2 low-level interface, run inside the simulation.
 *
 * Subscribes to `unitree_hg/LowCmd`, publishes `unitree_hg/LowState`, and closes
 * the per-motor PD loop the real robot closes in its motor drivers:
 *
 * @f[ \tau_i = k_{p,i}(q_{d,i} - q_i) + k_{d,i}(\dot q_{d,i} - \dot q_i) + \tau_{ff,i} @f]
 *
 * The same job a `controller_interface::ControllerInterface` would do over
 * ros2_control effort command interfaces, with two differences that motivate
 * doing it here instead:
 *
 * - The loop closes on every physics step (Execution::Step) rather than once per
 *   controller-manager period, which is what the real motor drivers do -- they
 *   run their PD far faster than the policy that sends the targets.
 * - The torque goes straight into `mjData::qfrc_applied`, so the joints need no
 *   command interfaces at all. ros2_control is then free to export state only,
 *   and the broadcasters keep working alongside this plugin.
 *
 * It is *not* a replacement for the controller on hardware: a controller runs
 * against the real robot unchanged, while this exists only in simulation. What it
 * replaces is the simulation half of that pair.
 *
 * @par Interaction with ros2_control
 * Remove the `<command_interface>` entries for these joints from the
 * `<ros2_control>` block. With none claimed, MujocoSystem's write() leaves their
 * `qfrc_applied` alone and there is nothing to fight over; leave one in and both
 * write the same field, last writer per step winning. The `<state_interface>`
 * entries stay: their `initial_value` is still what seeds the start pose, and
 * `joint_state_broadcaster` and the sensor broadcasters are unaffected.
 *
 * @par Configuration
 * Every value below is read as a parameter of this plugin's own node (so it
 * belongs in the same YAML as the controllers, keyed by the declaration name),
 * defaulting to the `<param>` of the same name on the declaration, defaulting to
 * the value given here.
 *
 * - `joints`                  : joints to drive, in LowCmd motor order (required)
 * - `motor_indices`           : `motor_cmd`/`motor_state` index per joint (default: 0..N-1)
 * - `effort_limits`           : per-joint torque clamp; one value applies to all,
 *                               empty means unclamped (default: empty)
 * - `imu_sensor_name`         : IMU site, naming the three MuJoCo sensors below;
 *                               empty leaves `imu_state` at zero (default: `imu_in_pelvis`)
 * - `imu_orientation_sensor`  : framequat sensor (default: `<imu_sensor_name>-orientation`)
 * - `imu_gyro_sensor`         : gyro sensor (default: `<imu_sensor_name>-angular-velocity`)
 * - `imu_accel_sensor`        : accelerometer sensor (default: `<imu_sensor_name>-linear-acceleration`)
 * - `command_topic`           : LowCmd topic (default: `/lowcmd`)
 * - `state_topic`             : LowState topic (default: `/lowstate`)
 * - `mode_machine`            : value echoed in `LowState::mode_machine`, which the
 *                               Unitree SDK examples copy back into LowCmd (default: 0)
 * - `rate`                    : LowState publish rate in simulated Hz (default: 500).
 *                               The one exception to the order above: a `<param
 *                               name="rate">` wins over the node parameter, because
 *                               the base class applies it before configure() runs.
 * - `joy_topic`, `joy.*`      : gamepad mapping, packed into
 *                               `LowState::wireless_remote` (default: `/joy`, unmapped)
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="unitree_g1_low_level"
 *                     plugin="mujoco_ros2_control_examples/UnitreeHgLowLevel">
 *   <param name="joints">left_hip_pitch_joint left_hip_roll_joint ...</param>
 * </mujoco_ros2_plugin>
 * @endcode
 *
 * @par Startup and reset
 * The joints are left slack -- zero torque, not held -- until the first LowCmd
 * arrives, and again after a reset, which is detected from `mjData::time` moving
 * backwards. That is what a robot whose motors are not yet enabled does, and it
 * keeps the plugin from inventing commands nobody sent: holding a pose is a
 * publisher's job, which is why the G1 example publishes its start pose on
 * /lowcmd at 50 Hz, keeping the robot standing until a policy publishes faster
 * and takes the topic over.
 */
class UnitreeHgLowLevel : public mujoco_ros2_control::MujocoRos2PluginInterface {
public:
    /// On the simulation thread: the PD loop wants every step, and it writes mjData.
    Execution execution() const override { return Execution::Step; }

    bool configure(
            const Context &context,
            const mujoco_ros2_control::MujocoRos2PluginInfo &info) override;

    /// Compute and apply the motor torques for the step about to be taken.
    void beforeStep(mjData *mujoco_data) override;

    /// Publish LowState for the state just integrated, at rate().
    void afterStep(const mjData *mujoco_data, const rclcpp::Time &stamp) override;

private:
    /// One driven joint: where it lives in mjModel, and which motor slot it is.
    struct Motor {
        std::string joint;
        int qpos_adr{-1};
        int dof_adr{-1};
        std::size_t index{0};       ///< index into LowCmd::motor_cmd / LowState::motor_state
        double effort_limit{0.0};   ///< 0 = unclamped
    };

    /// Resolve `joints` and `motor_indices` against the model. False on any error.
    bool bind_motors(const mjModel *mujoco_model);

    /// The PD torque for one joint, clamped to its effort limit.
    double torque(
            const Motor &motor, const mjData *mujoco_data, double kp, double kd,
            double target_position, double target_velocity, double feed_forward) const;

    /// Resolve the three IMU sensors. False only when one is declared but missing.
    bool bind_imu(const mjModel *mujoco_model);

    /// Pack a Joy message into the 40-byte Unitree wireless_remote layout (non-RT).
    std::array<uint8_t, 40> pack_joy(const sensor_msgs::msg::Joy &joy) const;

    /// Parameter readers: node parameter, then `<param>`, then @p fallback.
    std::string string_param(const char *key, const std::string &fallback);
    std::int64_t int_param(const char *key, std::int64_t fallback);
    double double_param(const char *key, double fallback);
    std::vector<std::string> string_array_param(const char *key);
    std::vector<std::int64_t> int_array_param(const char *key);
    std::vector<double> double_array_param(const char *key);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("unitree_hg_low_level")};

    std::vector<Motor> motors_;

    /// -1 when the sensor is not used; otherwise an offset into mjData::sensordata.
    int imu_quat_adr_{-1};
    int imu_gyro_adr_{-1};
    int imu_accel_adr_{-1};

    std::uint8_t mode_machine_{0};

    realtime_tools::RealtimeBuffer<unitree_hg::msg::LowCmd> command_buffer_;
    rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr command_sub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<unitree_hg::msg::LowState>> state_pub_;

    static constexpr std::size_t kWirelessRemoteSize = 40;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    realtime_tools::RealtimeBuffer<std::array<uint8_t, kWirelessRemoteSize>> joy_buffer_;
    std::atomic<bool> joy_received_{false};

    /// Joy.axes index per analog channel (-1 = unused, packed as 0.0).
    int axis_lx_{-1};
    int axis_ly_{-1};
    int axis_rx_{-1};
    int axis_ry_{-1};
    int axis_l2_{-1};
    double axis_lx_scale_{1.0};
    double axis_ly_scale_{1.0};
    double axis_rx_scale_{1.0};
    double axis_ry_scale_{1.0};
    double axis_l2_scale_{1.0};
    /// Parallel arrays: Joy.buttons[button_joy_indices_[k]] -> bit button_bits_[k].
    std::vector<std::int64_t> button_joy_indices_;
    std::vector<std::int64_t> button_bits_;

    /// Cleared by the first LowCmd, set again by a reset. While set, no torque.
    std::atomic<bool> holding_{true};
    /// Reset detection: mjData::time of the previous step, on the simulation thread.
    mjtNum last_sim_time_{-1.0};
    std::uint32_t tick_{0};
};

}  // namespace mujoco_ros2_control_examples

#endif  // MUJOCO_ROS2_CONTROL_EXAMPLES__UNITREE_HG_LOW_LEVEL_HPP_
