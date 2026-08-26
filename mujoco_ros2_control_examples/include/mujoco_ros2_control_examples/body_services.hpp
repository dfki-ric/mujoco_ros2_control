/**
 * @file body_services.hpp
 * @brief <mujoco_ros2_plugin> exposing per-body read and teleport services.
 *
 * @author Vamsi Origanti
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

#ifndef MUJOCO_ROS2_CONTROL_EXAMPLES__BODY_SERVICES_HPP_
#define MUJOCO_ROS2_CONTROL_EXAMPLES__BODY_SERVICES_HPP_

#include <memory>
#include <string>

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"
#include "mujoco_ros2_control_examples/srv/get_body_state.hpp"
#include "mujoco_ros2_control_examples/srv/set_body_pose.hpp"

namespace mujoco_ros2_control_examples {

/**
 * @brief Reads a body's pose and twist, and teleports a free body.
 *
 * Both services used to be built into the simulation node unconditionally, then
 * became a plugin shipped by mujoco_ros2_control. They live here now: teleporting
 * a body is a convenience for driving a simulation from outside, not part of
 * simulating one, so it belongs with the examples rather than in the core
 * package. Like TouchGridSensor, this doubles as a worked example of a
 * MujocoRos2PluginInterface implementation -- and of one that brings its own
 * service definitions (`srv/GetBodyState.srv`, `srv/SetBodyPose.srv` in this
 * package) rather than reusing an existing interface package.
 *
 * As a plugin the services are opt-in, so a model that does not need them does
 * not advertise them, and a description can declare them under whatever names
 * suit it.
 *
 * Parameters, both optional, and each read from this plugin's own node first
 * (keyed by the declaration name in the same YAML as the controllers), then from
 * the `<param>` of the same name on the declaration:
 * - `get_body_state_service` : service name (default `mujoco_get_body_state`)
 * - `set_body_pose_service`  : service name (default `mujoco_set_body_pose`)
 *
 * The defaults are the names the built-in services used. A node's name does not
 * prefix service names -- only its namespace does -- so declaring this plugin in
 * the root namespace serves exactly `/mujoco_get_body_state` and
 * `/mujoco_set_body_pose` as before, whatever the declaration is called. The
 * service *types* did move with the plugin, though: they are
 * mujoco_ros2_control_examples/srv/* now, not mujoco_ros2_control/srv/*.
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="body_services" plugin="mujoco_ros2_control_examples/BodyServices"/>
 * @endcode
 *
 * @par Why Execution::Step
 * Not for the step hooks -- it implements neither. It has no periodic work at
 * all, so it wants no thread of its own: everything happens in service
 * callbacks on the executor. Execution::Step is what says "do not start a
 * thread for me".
 *
 * @par Why the write is not deferred to beforeStep()
 * Queueing the pose and applying it in beforeStep() would be the tidier use of
 * the step hooks, but it breaks the case these services exist for. Under
 * `synchronous_mode` nothing advances the simulation except a StepSimulation
 * request, so a queued pose would not land until the client asked for a step --
 * while a client that sets a pose and then reads it back, with no step in
 * between, expects to see the pose it just set. So the write happens in the
 * callback, under both locks, and mj_forward() makes the derived state
 * (`xpos`/`xquat`, which the read service returns) consistent immediately.
 */
class BodyServices : public mujoco_ros2_control::MujocoRos2PluginInterface {
public:
    bool configure(
            const Context &context,
            const mujoco_ros2_control::MujocoRos2PluginInfo &info) override;

    /// No thread: this plugin only ever works inside its service callbacks.
    Execution execution() const override { return Execution::Step; }

private:
    void getBodyState(
            const std::shared_ptr<srv::GetBodyState::Request> request,
            std::shared_ptr<srv::GetBodyState::Response> response);

    void setBodyPose(
            const std::shared_ptr<srv::SetBodyPose::Request> request,
            std::shared_ptr<srv::SetBodyPose::Response> response);

    rclcpp::Service<srv::GetBodyState>::SharedPtr get_body_state_;
    rclcpp::Service<srv::SetBodyPose>::SharedPtr set_body_pose_;
};

}  // namespace mujoco_ros2_control_examples

#endif  // MUJOCO_ROS2_CONTROL_EXAMPLES__BODY_SERVICES_HPP_
