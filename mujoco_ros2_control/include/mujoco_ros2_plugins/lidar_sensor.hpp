/**
 * @file lidar_sensor.hpp
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

#ifndef MUJOCO_ROS2_PLUGINS__LIDAR_SENSOR_HPP_
#define MUJOCO_ROS2_PLUGINS__LIDAR_SENSOR_HPP_

#include <memory>

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"
#include "mujoco_gl_lidar/mujoco_gl_lidar.hpp"

namespace mujoco_ros2_plugins {

/**
 * @brief Declares a GL lidar through the robot description instead of a site prefix.
 *
 * A thin adapter: the scanning, rendering and publishing all stay in
 * mujoco_gl_lidar::MujocoGLLidar, which is unchanged and still reachable through
 * the site-prefix discovery in MujocoRos2Control. This only makes it declarable,
 * so a lidar can be named in the URDF next to the site it is mounted on rather
 * than being found by a naming convention.
 *
 * Parameters:
 * - `site` : the MuJoCo site the lidar is mounted on. Site +X is the 0-rad scan
 *            direction and it rotates about +Z. Defaults to the sensor name.
 *
 * Everything else -- frequency, ranges, angles, sample counts, noise, and
 * whether it publishes a LaserScan or a PointCloud2 -- is read by MujocoGLLidar
 * from this sensor's own node, so it is configured in a params YAML under the
 * sensor name, exactly as for a prefix-discovered lidar. The base class `rate`
 * parameter does not apply: this plugin runs the lidar's own loop, which paces
 * itself from its `frequency` parameter.
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="head" plugin="mujoco_ros2_control/LidarSensor">
 *   <param name="site">lidar_head</param>
 * </mujoco_ros2_plugin>
 * @endcode
 *
 * @note A site claimed by a declaration like this is skipped by the prefix
 *       discovery, so naming a site that also matches `site_prefix` yields one
 *       lidar rather than two.
 */
class LidarSensor : public mujoco_ros2_control::MujocoRos2PluginInterface {
public:
    bool configure(
            const Context &context,
            const mujoco_ros2_control::MujocoRos2PluginInfo &info) override;

    /// Hands the thread to MujocoGLLidar::update(), which owns its own pacing
    /// and takes the simulation mutex only for its per-scan mj_copyData.
    void run() override;

private:
    std::shared_ptr<mujoco_gl_lidar::MujocoGLLidar> lidar_;
};

}  // namespace mujoco_ros2_plugins

#endif  // MUJOCO_ROS2_PLUGINS__LIDAR_SENSOR_HPP_
