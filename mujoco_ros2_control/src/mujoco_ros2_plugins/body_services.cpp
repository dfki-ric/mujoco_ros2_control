/**
 * @file body_services.cpp
 * @brief <mujoco_ros2_plugin> exposing per-body read and teleport services.
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

#include "mujoco_ros2_plugins/body_services.hpp"

#include <mutex>

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_plugins {

using mujoco_ros2_control_plugins::get_param;

bool BodyServices::configure(
        const Context &context,
        const mujoco_ros2_control::MujocoRos2PluginInfo &info) {

    const std::string get_name =
        get_param(info, "get_body_state_service", "mujoco_get_body_state");
    const std::string set_name =
        get_param(info, "set_body_pose_service", "mujoco_set_body_pose");

    get_body_state_ = context.node->create_service<mujoco_ros2_control::srv::GetBodyState>(
        get_name,
        [this](const std::shared_ptr<mujoco_ros2_control::srv::GetBodyState::Request> request,
               std::shared_ptr<mujoco_ros2_control::srv::GetBodyState::Response> response) {
            getBodyState(request, response);
        });

    set_body_pose_ = context.node->create_service<mujoco_ros2_control::srv::SetBodyPose>(
        set_name,
        [this](const std::shared_ptr<mujoco_ros2_control::srv::SetBodyPose::Request> request,
               std::shared_ptr<mujoco_ros2_control::srv::SetBodyPose::Response> response) {
            setBodyPose(request, response);
        });

    RCLCPP_INFO(context.node->get_logger(), "Serving '%s' and '%s'",
                get_name.c_str(), set_name.c_str());
    return true;
}

void BodyServices::getBodyState(
        const std::shared_ptr<mujoco_ros2_control::srv::GetBodyState::Request> request,
        std::shared_ptr<mujoco_ros2_control::srv::GetBodyState::Response> response) {

    const int body_id =
        mj_name2id(context_.mujoco_model, mjOBJ_BODY, request->body_name.c_str());
    if (body_id < 0) {
        response->success = false;
        response->message = "Unknown body name: " + request->body_name;
        return;
    }

    // A read, so the inner lock alone is enough: it only has to not tear against
    // mj_step. Landing between two steps of a batch is fine, since every step in
    // the batch is a state the caller could legitimately observe.
    std::lock_guard<std::mutex> sim_lock(*context_.sim_mutex);
    const mjtNum *position = context_.mujoco_data->xpos + 3 * body_id;
    const mjtNum *quaternion = context_.mujoco_data->xquat + 4 * body_id;
    mjtNum velocity[6];
    mj_objectVelocity(
        context_.mujoco_model, context_.mujoco_data, mjOBJ_BODY, body_id, velocity, 0);

    response->pose.position.x = position[0];
    response->pose.position.y = position[1];
    response->pose.position.z = position[2];
    response->pose.orientation.w = quaternion[0];
    response->pose.orientation.x = quaternion[1];
    response->pose.orientation.y = quaternion[2];
    response->pose.orientation.z = quaternion[3];
    // mj_objectVelocity returns rotation first, then translation.
    response->twist.angular.x = velocity[0];
    response->twist.angular.y = velocity[1];
    response->twist.angular.z = velocity[2];
    response->twist.linear.x = velocity[3];
    response->twist.linear.y = velocity[4];
    response->twist.linear.z = velocity[5];
    response->success = true;
    response->message = "Body state returned in the world frame.";
}

void BodyServices::setBodyPose(
        const std::shared_ptr<mujoco_ros2_control::srv::SetBodyPose::Request> request,
        std::shared_ptr<mujoco_ros2_control::srv::SetBodyPose::Response> response) {

    const int body_id =
        mj_name2id(context_.mujoco_model, mjOBJ_BODY, request->body_name.c_str());
    if (body_id < 0) {
        response->success = false;
        response->message = "Unknown body name: " + request->body_name;
        return;
    }

    const int joint_address = context_.mujoco_model->body_jntadr[body_id];
    if (context_.mujoco_model->body_jntnum[body_id] != 1 ||
        context_.mujoco_model->jnt_type[joint_address] != mjJNT_FREE) {
        response->success = false;
        response->message =
            "Body '" + request->body_name + "' does not have a single free joint.";
        return;
    }

    const int qpos_address = context_.mujoco_model->jnt_qposadr[joint_address];
    const int dof_address = context_.mujoco_model->jnt_dofadr[joint_address];

    // Both locks, outer first: a teleport must not land in the middle of a
    // StepSimulation batch or a reset, which is what step_mutex serialises.
    std::lock_guard<std::mutex> step_lock(*context_.step_mutex);
    std::lock_guard<std::mutex> sim_lock(*context_.sim_mutex);

    mjtNum *qpos = context_.mujoco_data->qpos + qpos_address;
    qpos[0] = request->x;
    qpos[1] = request->y;
    qpos[2] = request->z;
    qpos[3] = request->qw;
    qpos[4] = request->qx;
    qpos[5] = request->qy;
    qpos[6] = request->qz;
    for (int i = 0; i < 6; i++) {
        context_.mujoco_data->qvel[dof_address + i] = 0.0;
    }
    // Makes xpos/xquat agree with the new qpos without waiting for a step, so a
    // client can read the pose straight back even in synchronous mode.
    mj_forward(context_.mujoco_model, context_.mujoco_data);

    response->success = true;
    response->message = "Body '" + request->body_name + "' repositioned.";
}

}  // namespace mujoco_ros2_plugins

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_plugins::BodyServices, mujoco_ros2_control::MujocoRos2PluginInterface)
