/**
 * @file depth_camera_sensor.cpp
 * @brief <mujoco_ros2_plugin> plugin wrapping MujocoDepthCamera.
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

#include "mujoco_ros2_plugins/depth_camera_sensor.hpp"

#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_plugins {

using mujoco_ros2_control_plugins::get_param;

bool DepthCameraSensor::configure(
        const Context &context,
        const mujoco_ros2_control::MujocoRos2PluginInfo &info) {

    const auto logger = context.node->get_logger();

    auto resolve_site = [&](const std::string &site_name) {
        const int id = mj_name2id(context.mujoco_model, mjOBJ_SITE, site_name.c_str());
        if (id < 0) {
            RCLCPP_ERROR(logger, "The model has no site '%s'.", site_name.c_str());
        }
        return id;
    };

    auto mount = mujoco_rgbd_camera::MujocoDepthCamera::Mount::Site;
    int optical_id = -1;
    int depth_id = -1;
    std::string mounted_on;

    const std::string camera_name = get_param(info, "camera");
    const std::string site = get_param(info, "site");
    const std::string optical_site = get_param(info, "optical_site");
    const std::string depth_site = get_param(info, "depth_site");

    if (!camera_name.empty()) {
        if (!site.empty() || !optical_site.empty() || !depth_site.empty()) {
            RCLCPP_ERROR(logger,
                "'camera' cannot be combined with 'site', 'optical_site' or 'depth_site': "
                "a camera is mounted either on a <camera> element or on sites, not both.");
            return false;
        }
        const int camera_id = mj_name2id(context.mujoco_model, mjOBJ_CAMERA, camera_name.c_str());
        if (camera_id < 0) {
            RCLCPP_ERROR(logger, "The model has no <camera> named '%s'.", camera_name.c_str());
            return false;
        }
        // One viewpoint serves both streams, so both mount ids are the camera id.
        mount = mujoco_rgbd_camera::MujocoDepthCamera::Mount::FixedCamera;
        optical_id = camera_id;
        depth_id = camera_id;
        mounted_on = "camera '" + camera_name + "'";

    } else if (!site.empty() && (!optical_site.empty() || !depth_site.empty())) {
        RCLCPP_ERROR(logger,
            "'site' names one frame for both streams, so it cannot be combined with "
            "'optical_site' or 'depth_site'.");
        return false;

    } else if (!optical_site.empty() || !depth_site.empty()) {
        // Two-frame camera: the streams are rendered from separate sites, and
        // either one may be absent.
        if (!optical_site.empty()) {
            optical_id = resolve_site(optical_site);
            if (optical_id < 0) {
                return false;
            }
        }
        if (!depth_site.empty()) {
            depth_id = resolve_site(depth_site);
            if (depth_id < 0) {
                return false;
            }
        }
        mounted_on = "optical site '" + (optical_site.empty() ? std::string("-") : optical_site) +
                     "', depth site '" + (depth_site.empty() ? std::string("-") : depth_site) + "'";

    } else {
        // A single site for both streams; the sensor name is the fallback.
        const std::string both = site.empty() ? info.name : site;
        const int id = resolve_site(both);
        if (id < 0) {
            return false;
        }
        optical_id = id;
        depth_id = id;
        mounted_on = "site '" + both + "'";
    }

    // MujocoDepthCamera takes the node by non-const reference and sets up EGL in
    // its constructor, which can throw.
    auto node = context.node;
    try {
        camera_ = std::make_shared<mujoco_rgbd_camera::MujocoDepthCamera>(
            node, context.mujoco_model, context.mujoco_data, context.sim_mutex,
            info.name, context.stop, mount, optical_id, depth_id);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Could not bring up the camera on %s: %s",
                     mounted_on.c_str(), e.what());
        return false;
    }

    RCLCPP_INFO(logger, "Camera mounted on %s", mounted_on.c_str());
    return true;
}

void DepthCameraSensor::run() { camera_->update(); }

std::uint64_t DepthCameraSensor::requestSynchronousFrame() {
    return camera_->request_synchronous_frame();
}

bool DepthCameraSensor::waitForSynchronousFrame(
        std::uint64_t sequence, std::chrono::milliseconds timeout) {
    return camera_->wait_for_synchronous_frame(sequence, timeout);
}

}  // namespace mujoco_ros2_plugins

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_plugins::DepthCameraSensor, mujoco_ros2_control::MujocoRos2PluginInterface)
