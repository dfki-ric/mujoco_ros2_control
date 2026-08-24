/**
 * @file pose_sensor.cpp
 * @brief Pose sensor plugin: frame position and orientation.
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

#include "mujoco_ros2_control_plugins/pose_sensor.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_control_plugins {

bool PoseSensor::registerSensor(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::ComponentInfo &sensor_info,
        std::vector<hardware_interface::StateInterface> &state_interfaces) {
    logger_ = node->get_logger().get_child(name_);

    object_name_ = resolve_object_name(sensor_info);

    framepos_adr_ = find_sensor_adr(mujoco_model, object_name_, mjSENS_FRAMEPOS);
    framequat_adr_ = find_sensor_adr(mujoco_model, object_name_, mjSENS_FRAMEQUAT);

    if (framepos_adr_ < 0 && framequat_adr_ < 0) {
        RCLCPP_ERROR(logger_,
            "No framepos or framequat sensor is attached to '%s' in the MJCF.",
            object_name_.c_str());
        return false;
    }

    export_state_interfaces(sensor_info, state_interfaces, {
        {"position.x", &position_x_},
        {"position.y", &position_y_},
        {"position.z", &position_z_},
        {"orientation.x", &orientation_x_},
        {"orientation.y", &orientation_y_},
        {"orientation.z", &orientation_z_},
        {"orientation.w", &orientation_w_},
    });

    RCLCPP_INFO(logger_, "Pose bound to '%s' (framepos %s, framequat %s)",
                object_name_.c_str(),
                framepos_adr_ >= 0 ? "yes" : "no",
                framequat_adr_ >= 0 ? "yes" : "no");
    return true;
}

void PoseSensor::read(const mjData *mujoco_data) {
    if (framepos_adr_ >= 0) {
        position_x_ = mujoco_data->sensordata[framepos_adr_];
        position_y_ = mujoco_data->sensordata[framepos_adr_ + 1];
        position_z_ = mujoco_data->sensordata[framepos_adr_ + 2];
    }
    if (framequat_adr_ >= 0) {
        // MuJoCo stores quaternions w-first, ROS x-first.
        orientation_w_ = mujoco_data->sensordata[framequat_adr_];
        orientation_x_ = mujoco_data->sensordata[framequat_adr_ + 1];
        orientation_y_ = mujoco_data->sensordata[framequat_adr_ + 2];
        orientation_z_ = mujoco_data->sensordata[framequat_adr_ + 3];
    }
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_control_plugins::PoseSensor, mujoco_ros2_control::MujocoSensorInterface)
