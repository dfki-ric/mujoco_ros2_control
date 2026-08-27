/**
 * @file depth_camera_sensor.hpp
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

#ifndef MUJOCO_ROS2_PLUGINS__DEPTH_CAMERA_SENSOR_HPP_
#define MUJOCO_ROS2_PLUGINS__DEPTH_CAMERA_SENSOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"
#include "mujoco_ros2_plugins/depth_camera_impl.hpp"

namespace mujoco_ros2_plugins {

/**
 * @brief Declares an RGB-D camera through the robot description instead of a
 *        `<camera>` element or a site prefix.
 *
 * A thin adapter: the rendering and publishing all stay in
 * mujoco_ros2_plugins_depth_camera::DepthCameraImpl, a standalone copy of
 * mujoco_rgbd_camera::MujocoDepthCamera kept independent of the folder used by
 * the site-prefix discovery in MujocoRos2Control. This only makes it declarable,
 * so a camera can be named in the URDF next to the frame it looks from rather
 * than being found by a naming convention.
 *
 * Exactly one mount has to be named:
 * - `camera`       : a MuJoCo `<camera>` element. Pose and vertical FOV come
 *                    from the model camera, and one render pass serves both the
 *                    color and the depth stream.
 * - `site`         : one site for both streams, read as a REP-103 optical frame
 *                    (+Z forward, +X right, +Y down), with the vertical FOV from
 *                    the `fovy` parameter.
 * - `optical_site` and/or `depth_site` : the two frames of a two-frame camera,
 *                    rendered separately. Either may be left out for a camera
 *                    with only one of the two streams.
 *
 * With none of them set the sensor name is used as a site name.
 *
 * Everything else -- resolution, frequency, fovy, what gets published -- is read
 * by MujocoDepthCamera from this sensor's own node, so it is configured in a
 * params YAML under the sensor name, exactly as for a discovered camera. The
 * base class `rate` parameter does not apply: this plugin runs the camera's own
 * loop, which paces itself from its `frequency` parameter.
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="wrist_cam" plugin="mujoco_ros2_control/DepthCameraSensor">
 *   <param name="site">wrist_cam_optical</param>
 * </mujoco_ros2_plugin>
 * @endcode
 *
 * @note A site claimed by a declaration like this is skipped by the prefix
 *       discovery, so naming a site that also matches one of the camera site
 *       prefixes yields one camera rather than two.
 */
class DepthCameraSensor : public mujoco_ros2_control::MujocoRos2PluginInterface {
public:
    bool configure(
            const Context &context,
            const mujoco_ros2_control::MujocoRos2PluginInfo &info) override;

    /// Hands the thread to MujocoDepthCamera::update(), which owns its own
    /// pacing and its own snapshotting of mjData.
    void run() override;

    /// Forwarded to the camera so deterministic stepping waits for this camera's
    /// post-step frame. Without these a declared camera is left out of the
    /// StepSimulation barrier that discovery-created cameras get, and a client
    /// can read an image from a state it has already stepped past.
    std::uint64_t requestSynchronousFrame() override;
    bool waitForSynchronousFrame(
            std::uint64_t sequence, std::chrono::milliseconds timeout) override;

private:
    std::shared_ptr<mujoco_ros2_plugins_depth_camera::DepthCameraImpl> camera_;
};

}  // namespace mujoco_ros2_plugins

#endif  // MUJOCO_ROS2_PLUGINS__DEPTH_CAMERA_SENSOR_HPP_
