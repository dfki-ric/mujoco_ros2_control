/**
 * @file lidar_sensor.cpp
 * @brief <mujoco_ros2_plugin> plugin wrapping MujocoGLLidar.
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

#include "mujoco_ros2_plugins/lidar_sensor.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_plugins {

bool LidarSensor::configure(
        const Context &context,
        const mujoco_ros2_control::MujocoRos2PluginInfo &info) {

    const std::string site_name =
        mujoco_ros2_control_plugins::get_param(info, "site", info.name);

    const int site_id = mj_name2id(context.mujoco_model, mjOBJ_SITE, site_name.c_str());
    if (site_id < 0) {
        RCLCPP_ERROR(context.node->get_logger(),
            "The model has no site '%s' to mount the lidar on.", site_name.c_str());
        return false;
    }

    // MujocoGLLidar takes the node by non-const reference and reads its
    // parameters in the constructor, which can throw once EGL is involved.
    auto node = context.node;
    try {
        lidar_ = std::make_shared<mujoco_ros2_plugins_lidar::LidarImpl>(
            node, context.mujoco_model, context.mujoco_data, context.sim_mutex,
            site_id, info.name, context.stop);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(context.node->get_logger(),
            "Could not bring up the lidar on site '%s': %s", site_name.c_str(), e.what());
        return false;
    }

    RCLCPP_INFO(context.node->get_logger(), "Lidar mounted on site '%s'", site_name.c_str());
    return true;
}

void LidarSensor::run() { lidar_->update(); }

}  // namespace mujoco_ros2_plugins

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_plugins::LidarSensor, mujoco_ros2_control::MujocoRos2PluginInterface)
