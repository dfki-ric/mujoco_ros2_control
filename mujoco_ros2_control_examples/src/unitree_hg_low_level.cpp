/**
 * @file unitree_hg_low_level.cpp
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

#include "mujoco_ros2_control_examples/unitree_hg_low_level.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_control_examples {

// Shared parameter readers, exported by mujoco_ros2_control alongside the base
// class, so this plugin reads its configuration the way the built-in ones do.
using mujoco_ros2_control_plugins::declare_double_array_param;
using mujoco_ros2_control_plugins::declare_double_param;
using mujoco_ros2_control_plugins::declare_int_array_param;
using mujoco_ros2_control_plugins::declare_int_param;
using mujoco_ros2_control_plugins::declare_param;
using mujoco_ros2_control_plugins::declare_string_array_param;

namespace {

/// unitree_hg LowCmd/LowState carry fixed-size motor arrays.
constexpr std::size_t kMotorArraySize = 35;

/// Unitree wireless_remote (xRockerBtnDataStruct) byte offsets in the 40-byte blob.
constexpr std::size_t kWrHeaderOffset = 0;   // 2 bytes
constexpr std::size_t kWrKeyOffset = 2;      // uint16 button bitmask
constexpr std::size_t kWrLxOffset = 4;       // float
constexpr std::size_t kWrRxOffset = 8;       // float
constexpr std::size_t kWrRyOffset = 12;      // float
constexpr std::size_t kWrL2Offset = 16;      // float
constexpr std::size_t kWrLyOffset = 20;      // float

/// Pack one analog channel (Joy.axes[index] * scale) as a little-endian float.
void pack_axis(
        std::array<uint8_t, 40> &buffer, std::size_t offset, int index, double scale,
        const sensor_msgs::msg::Joy &joy) {
    float value = 0.0f;
    if (index >= 0 && static_cast<std::size_t>(index) < joy.axes.size()) {
        value = static_cast<float>(joy.axes[static_cast<std::size_t>(index)] * scale);
    }
    std::memcpy(buffer.data() + offset, &value, sizeof(float));
}

}  // namespace

//
// Parameter readers. Thin wrappers around the shared ones, which resolve the
// node parameter first, then the `<param>` of the same name on the declaration,
// then the fallback given here -- so this robot can be described entirely in its
// URDF and still be retuned from the YAML that configures the controllers. The
// wrappers exist only to keep the node and the declaration out of every call.
//

std::string UnitreeHgLowLevel::string_param(const char *key, const std::string &fallback) {
    return declare_param(node_, info_, key, fallback);
}

std::int64_t UnitreeHgLowLevel::int_param(const char *key, std::int64_t fallback) {
    return declare_int_param(node_, info_, key, fallback);
}

double UnitreeHgLowLevel::double_param(const char *key, double fallback) {
    return declare_double_param(node_, info_, key, fallback);
}

std::vector<std::string> UnitreeHgLowLevel::string_array_param(const char *key) {
    return declare_string_array_param(node_, info_, key);
}

std::vector<std::int64_t> UnitreeHgLowLevel::int_array_param(const char *key) {
    return declare_int_array_param(node_, info_, key);
}

std::vector<double> UnitreeHgLowLevel::double_array_param(const char *key) {
    return declare_double_array_param(node_, info_, key);
}

bool UnitreeHgLowLevel::bind_motors(const mjModel *mujoco_model) {
    const auto joints = string_array_param("joints");
    if (joints.empty()) {
        RCLCPP_ERROR(logger_,
            "No 'joints' configured. List the joints to drive, in the order their "
            "motor indices are given, either as a <param name=\"joints\"> on the "
            "declaration or as a parameter of this node.");
        return false;
    }

    auto indices = int_array_param("motor_indices");
    if (indices.empty()) {
        indices.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); i++) {
            indices[i] = static_cast<std::int64_t>(i);
        }
    }
    if (indices.size() != joints.size()) {
        RCLCPP_ERROR(logger_,
            "'motor_indices' has %zu entries but 'joints' has %zu.",
            indices.size(), joints.size());
        return false;
    }

    const auto limits = double_array_param("effort_limits");
    if (limits.size() > 1 && limits.size() != joints.size()) {
        RCLCPP_ERROR(logger_,
            "'effort_limits' has %zu entries: it has to be empty (unclamped), hold "
            "one value for every joint, or hold a single value used for all %zu.",
            limits.size(), joints.size());
        return false;
    }

    motors_.clear();
    motors_.reserve(joints.size());
    for (std::size_t i = 0; i < joints.size(); i++) {
        Motor motor;
        motor.joint = joints[i];

        const int joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, motor.joint.c_str());
        if (joint_id < 0) {
            RCLCPP_ERROR(logger_, "The model has no joint '%s'.", motor.joint.c_str());
            return false;
        }
        const int joint_type = mujoco_model->jnt_type[joint_id];
        if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
            RCLCPP_ERROR(logger_,
                "Joint '%s' is not a hinge or slide, so it has no single torque to "
                "command.", motor.joint.c_str());
            return false;
        }

        if (indices[i] < 0 || static_cast<std::size_t>(indices[i]) >= kMotorArraySize) {
            RCLCPP_ERROR(logger_,
                "Motor index %ld for joint '%s' is outside the LowCmd motor array [0, %zu).",
                indices[i], motor.joint.c_str(), kMotorArraySize);
            return false;
        }

        motor.qpos_adr = mujoco_model->jnt_qposadr[joint_id];
        motor.dof_adr = mujoco_model->jnt_dofadr[joint_id];
        motor.index = static_cast<std::size_t>(indices[i]);
        if (limits.size() == 1) {
            motor.effort_limit = std::abs(limits.front());
        } else if (!limits.empty()) {
            motor.effort_limit = std::abs(limits[i]);
        }
        motors_.push_back(motor);
    }

    return true;
}

double UnitreeHgLowLevel::torque(
        const Motor &motor, const mjData *mujoco_data, double kp, double kd,
        double target_position, double target_velocity, double feed_forward) const {
    const double position = mujoco_data->qpos[motor.qpos_adr];
    const double velocity = mujoco_data->qvel[motor.dof_adr];

    double value = kp * (target_position - position) + kd * (target_velocity - velocity) +
                   feed_forward;
    if (motor.effort_limit > 0.0) {
        value = std::clamp(value, -motor.effort_limit, motor.effort_limit);
    }
    return value;
}

bool UnitreeHgLowLevel::bind_imu(const mjModel *mujoco_model) {
    const std::string imu = string_param("imu_sensor_name", "imu_in_pelvis");

    // The three sensors are named after the site by the convention the built-in
    // ImuSensor uses, so naming the site is normally all that is needed.
    const std::string quat = string_param(
        "imu_orientation_sensor", imu.empty() ? "" : imu + "-orientation");
    const std::string gyro = string_param(
        "imu_gyro_sensor", imu.empty() ? "" : imu + "-angular-velocity");
    const std::string accel = string_param(
        "imu_accel_sensor", imu.empty() ? "" : imu + "-linear-acceleration");

    struct Binding {
        const std::string &name;
        int dimension;
        int *address;
    };
    for (const auto &binding : {Binding{quat, 4, &imu_quat_adr_},
                                Binding{gyro, 3, &imu_gyro_adr_},
                                Binding{accel, 3, &imu_accel_adr_}}) {
        if (binding.name.empty()) {
            continue;
        }
        const int id = mj_name2id(mujoco_model, mjOBJ_SENSOR, binding.name.c_str());
        if (id < 0) {
            RCLCPP_ERROR(logger_,
                "The model has no sensor '%s'. Add it to the <sensor> block, or set "
                "'imu_sensor_name' to \"\" to publish LowState without an IMU.",
                binding.name.c_str());
            return false;
        }
        if (mujoco_model->sensor_dim[id] != binding.dimension) {
            RCLCPP_ERROR(logger_,
                "Sensor '%s' produces %d values, expected %d.",
                binding.name.c_str(), mujoco_model->sensor_dim[id], binding.dimension);
            return false;
        }
        *binding.address = mujoco_model->sensor_adr[id];
    }

    if (imu_quat_adr_ < 0 && imu_gyro_adr_ < 0 && imu_accel_adr_ < 0) {
        RCLCPP_WARN(logger_, "No IMU sensor configured: LowState::imu_state stays zero.");
    }
    return true;
}

bool UnitreeHgLowLevel::configure(
        const Context &context,
        const mujoco_ros2_control::MujocoRos2PluginInfo &info) {
    // Read through info_ rather than info: initialize() stored it before calling
    // this, and the parameter readers below need it after configure() returns.
    (void)info;
    node_ = context.node;
    logger_ = node_->get_logger();

    // The motor drivers publish state far faster than a policy needs, but 500 Hz
    // is what the G1 sends over DDS.
    set_default_rate(500.0);
    set_default_rate(double_param("rate", rate()));

    if (!bind_motors(context.mujoco_model) || !bind_imu(context.mujoco_model)) {
        return false;
    }

    mode_machine_ = static_cast<std::uint8_t>(int_param("mode_machine", 0));

    const auto command_topic = string_param("command_topic", "/lowcmd");
    const auto state_topic = string_param("state_topic", "/lowstate");

    // Joy -> wireless_remote mapping. Unmapped channels are packed as zero, so a
    // policy that reads the gamepad still gets a well-formed blob.
    const auto joy_topic = string_param("joy_topic", "/joy");
    axis_lx_ = static_cast<int>(int_param("joy.axis_lx", -1));
    axis_ly_ = static_cast<int>(int_param("joy.axis_ly", -1));
    axis_rx_ = static_cast<int>(int_param("joy.axis_rx", -1));
    axis_ry_ = static_cast<int>(int_param("joy.axis_ry", -1));
    axis_l2_ = static_cast<int>(int_param("joy.axis_l2", -1));
    axis_lx_scale_ = double_param("joy.axis_lx_scale", 1.0);
    axis_ly_scale_ = double_param("joy.axis_ly_scale", 1.0);
    axis_rx_scale_ = double_param("joy.axis_rx_scale", 1.0);
    axis_ry_scale_ = double_param("joy.axis_ry_scale", 1.0);
    axis_l2_scale_ = double_param("joy.axis_l2_scale", 1.0);
    button_joy_indices_ = int_array_param("joy.button_joy_indices");
    button_bits_ = int_array_param("joy.button_bits");
    if (button_joy_indices_.size() != button_bits_.size()) {
        RCLCPP_ERROR(logger_,
            "'joy.button_joy_indices' has %zu entries but 'joy.button_bits' has %zu.",
            button_joy_indices_.size(), button_bits_.size());
        return false;
    }
    for (const auto bit : button_bits_) {
        if (bit < 0 || bit > 15) {
            RCLCPP_ERROR(logger_,
                "'joy.button_bits' holds %ld, outside the 16-bit key mask [0, 15].", bit);
            return false;
        }
    }

    command_buffer_.writeFromNonRT(unitree_hg::msg::LowCmd());
    joy_buffer_.writeFromNonRT(std::array<uint8_t, kWirelessRemoteSize>{});

    // These callbacks run on the loader's executor, not on the simulation thread,
    // so everything they hand over goes through a realtime buffer.
    command_sub_ = node_->create_subscription<unitree_hg::msg::LowCmd>(
        command_topic, rclcpp::SystemDefaultsQoS(),
        [this](const unitree_hg::msg::LowCmd::SharedPtr message) {
            command_buffer_.writeFromNonRT(*message);
            holding_.store(false, std::memory_order_release);
        });

    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr message) {
            joy_buffer_.writeFromNonRT(pack_joy(*message));
            joy_received_.store(true, std::memory_order_release);
        });

    auto publisher = node_->create_publisher<unitree_hg::msg::LowState>(
        state_topic, rclcpp::SystemDefaultsQoS());
    state_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<unitree_hg::msg::LowState>>(publisher);

    RCLCPP_INFO(logger_,
        "Driving %zu joints from '%s', publishing '%s' at %.1f Hz. The joints stay "
        "slack until the first LowCmd arrives.",
        motors_.size(), command_topic.c_str(), state_topic.c_str(), rate());
    return true;
}

void UnitreeHgLowLevel::beforeStep(mjData *mujoco_data) {
    // A reset rewinds mjData::time, and the pose it lands in is not one the last
    // command was computed for: go slack and wait for the policy to catch up,
    // exactly as at startup.
    if (mujoco_data->time < last_sim_time_) {
        holding_.store(true, std::memory_order_release);
        tick_ = 0;
    }
    last_sim_time_ = mujoco_data->time;

    // qfrc_applied is not cleared by MuJoCo, so every step has to write it --
    // including the steps that apply nothing, or the last torque keeps acting.
    if (holding_.load(std::memory_order_acquire)) {
        for (const auto &motor : motors_) {
            mujoco_data->qfrc_applied[motor.dof_adr] = 0.0;
        }
        return;
    }

    // The PD loop the real motor driver closes, at physics rate rather than at
    // the rate the targets arrive.
    const auto &command = *command_buffer_.readFromRT();
    for (const auto &motor : motors_) {
        const auto &motor_cmd = command.motor_cmd[motor.index];
        mujoco_data->qfrc_applied[motor.dof_adr] = torque(
            motor, mujoco_data, motor_cmd.kp, motor_cmd.kd, motor_cmd.q, motor_cmd.dq,
            motor_cmd.tau);
    }
}

void UnitreeHgLowLevel::afterStep(const mjData *mujoco_data, const rclcpp::Time &stamp) {
    (void)stamp;  // LowState carries a tick rather than a stamp.

    if (!due(mujoco_data->time)) {
        return;
    }
    // trylock() rather than lock(): this runs on the simulation thread, so waiting
    // on the publishing thread here would stall mj_step().
    if (!state_pub_ || !state_pub_->trylock()) {
        return;
    }

    auto &state = state_pub_->msg_;
    state.tick = tick_++;
    state.mode_machine = mode_machine_;
    state.mode_pr = command_buffer_.readFromRT()->mode_pr;

    for (const auto &motor : motors_) {
        auto &motor_state = state.motor_state[motor.index];
        motor_state.q = static_cast<float>(mujoco_data->qpos[motor.qpos_adr]);
        motor_state.dq = static_cast<float>(mujoco_data->qvel[motor.dof_adr]);
        motor_state.ddq = static_cast<float>(mujoco_data->qacc[motor.dof_adr]);
        // What the joint actually saw: the applied torque plus anything an
        // actuator in the model contributed. Same quantity MujocoSystem reports
        // as the effort state interface.
        motor_state.tau_est = static_cast<float>(
            mujoco_data->qfrc_actuator[motor.dof_adr] + mujoco_data->qfrc_applied[motor.dof_adr]);
    }

    if (imu_quat_adr_ >= 0) {
        const mjtNum *quaternion = mujoco_data->sensordata + imu_quat_adr_;
        // Both a MuJoCo framequat and unitree_hg IMUState order it [w, x, y, z],
        // so this is a copy rather than the reshuffle the ros2_control IMU state
        // interfaces (x, y, z, w) need.
        for (std::size_t i = 0; i < 4; i++) {
            state.imu_state.quaternion[i] = static_cast<float>(quaternion[i]);
        }

        const double w = quaternion[0];
        const double x = quaternion[1];
        const double y = quaternion[2];
        const double z = quaternion[3];
        state.imu_state.rpy[0] =
            static_cast<float>(std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)));
        state.imu_state.rpy[1] =
            static_cast<float>(std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)));
        state.imu_state.rpy[2] =
            static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
    }
    if (imu_gyro_adr_ >= 0) {
        const mjtNum *gyroscope = mujoco_data->sensordata + imu_gyro_adr_;
        for (std::size_t i = 0; i < 3; i++) {
            state.imu_state.gyroscope[i] = static_cast<float>(gyroscope[i]);
        }
    }
    if (imu_accel_adr_ >= 0) {
        const mjtNum *accelerometer = mujoco_data->sensordata + imu_accel_adr_;
        for (std::size_t i = 0; i < 3; i++) {
            state.imu_state.accelerometer[i] = static_cast<float>(accelerometer[i]);
        }
    }

    if (joy_received_.load(std::memory_order_acquire)) {
        const auto &wireless_remote = *joy_buffer_.readFromRT();
        std::copy(wireless_remote.begin(), wireless_remote.end(), state.wireless_remote.begin());
    }

    state_pub_->unlockAndPublish();
}

std::array<uint8_t, 40> UnitreeHgLowLevel::pack_joy(const sensor_msgs::msg::Joy &joy) const {
    std::array<uint8_t, 40> buffer{};

    // Header of the Unitree SDK wireless packet.
    buffer[kWrHeaderOffset] = 0xFE;
    buffer[kWrHeaderOffset + 1] = 0xEF;

    pack_axis(buffer, kWrLxOffset, axis_lx_, axis_lx_scale_, joy);
    pack_axis(buffer, kWrLyOffset, axis_ly_, axis_ly_scale_, joy);
    pack_axis(buffer, kWrRxOffset, axis_rx_, axis_rx_scale_, joy);
    pack_axis(buffer, kWrRyOffset, axis_ry_, axis_ry_scale_, joy);
    pack_axis(buffer, kWrL2Offset, axis_l2_, axis_l2_scale_, joy);

    uint16_t keys = 0;
    for (std::size_t k = 0; k < button_joy_indices_.size(); k++) {
        const auto index = button_joy_indices_[k];
        if (index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size() &&
            joy.buttons[static_cast<std::size_t>(index)] != 0) {
            keys |= static_cast<uint16_t>(1u << button_bits_[k]);
        }
    }
    std::memcpy(buffer.data() + kWrKeyOffset, &keys, sizeof(uint16_t));

    return buffer;
}

}  // namespace mujoco_ros2_control_examples

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_control_examples::UnitreeHgLowLevel, mujoco_ros2_control::MujocoRos2PluginInterface)
