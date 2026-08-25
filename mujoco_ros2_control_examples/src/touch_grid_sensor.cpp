/**
 * @file touch_grid_sensor.cpp
 * @brief Sensor plugin publishing MuJoCo's touch_grid tactile array to a topic.
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

#include "mujoco_ros2_control_examples/touch_grid_sensor.hpp"

#include <sstream>

#include "pluginlib/class_list_macros.hpp"

#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace mujoco_ros2_control_examples {

// Shared <param> readers, exported by mujoco_ros2_control alongside the base
// class. Reused rather than reimplemented so an out-of-tree plugin reads its
// configuration exactly the way the built-in ones do.
using mujoco_ros2_control_plugins::get_bool_param;
using mujoco_ros2_control_plugins::get_param;

bool TouchGridSensor::configure(
        const Context &context,
        const mujoco_ros2_control::MujocoRos2PluginInfo &info) {
    const mjModel *mujoco_model = context.mujoco_model;

    logger_ = context.node->get_logger();

    site_name_ = get_param(info, "site", info.name);
    // Left relative to the node's namespace rather than made private to the
    // node: a node name does not prefix topics, so "<name>/touch_grid" is what
    // keeps the grid on /<name>/touch_grid.
    topic_ = get_param(info, "topic", info.name + "/touch_grid");
    frame_id_ = get_param(info, "frame_id", site_name_);
    publish_ = get_bool_param(info, "publish", true);

    int plugin_instance = -1;
    for (int id = 0; id < mujoco_model->nsensor; id++) {
        if (mujoco_model->sensor_type[id] != mjSENS_PLUGIN) continue;
        if (mujoco_model->sensor_objtype[id] != mjOBJ_SITE) continue;

        const char *site = mj_id2name(mujoco_model, mjOBJ_SITE, mujoco_model->sensor_objid[id]);
        if (!site || site_name_ != site) continue;

        sensor_adr_ = mujoco_model->sensor_adr[id];
        sensor_dim_ = mujoco_model->sensor_dim[id];
        plugin_instance = mujoco_model->sensor_plugin[id];
        break;
    }

    if (plugin_instance < 0) {
        RCLCPP_ERROR(logger_,
            "No touch_grid plugin sensor is attached to site '%s'. The MJCF needs a "
            "<plugin plugin=\"mujoco.sensor.touch_grid\" objtype=\"site\" objname=\"%s\"/> entry, "
            "and the touch_grid plugin library has to be loaded.",
            site_name_.c_str(), site_name_.c_str());
        return false;
    }

    // The grid shape lives in the engine plugin's config rather than in mjModel,
    // so it has to be read back to label the message dimensions.
    if (!read_int_config(mujoco_model, plugin_instance, "nchannel", 1, &nchannel_) ||
        !read_size_config(mujoco_model, plugin_instance)) {
        return false;
    }

    if (nchannel_ * width_ * height_ != sensor_dim_) {
        RCLCPP_ERROR(logger_,
            "touch_grid config (%d channels, %dx%d) does not match the sensor's %d outputs.",
            nchannel_, width_, height_, sensor_dim_);
        return false;
    }

    data_.assign(static_cast<size_t>(sensor_dim_), 0.0);

    if (!publish_) {
        RCLCPP_INFO(logger_,
            "Stepping a %d x %d x %d touch_grid on site '%s' with publishing disabled.",
            nchannel_, height_, width_, site_name_.c_str());
        return true;
    }

    auto publisher = context.node->create_publisher<std_msgs::msg::Float64MultiArray>(
        topic_, rclcpp::SensorDataQoS());
    realtime_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
            publisher);

    // The layout is fixed for the lifetime of the model, so fill it once rather
    // than rebuilding it on every control cycle.
    auto &msg = realtime_publisher_->msg_;
    msg.layout.dim.resize(3);
    msg.layout.dim[0].label = "channel";
    msg.layout.dim[0].size = static_cast<uint32_t>(nchannel_);
    msg.layout.dim[0].stride = static_cast<uint32_t>(sensor_dim_);
    msg.layout.dim[1].label = "height";
    msg.layout.dim[1].size = static_cast<uint32_t>(height_);
    msg.layout.dim[1].stride = static_cast<uint32_t>(width_ * height_);
    msg.layout.dim[2].label = "width";
    msg.layout.dim[2].size = static_cast<uint32_t>(width_);
    msg.layout.dim[2].stride = static_cast<uint32_t>(width_);
    msg.data.assign(static_cast<size_t>(sensor_dim_), 0.0);

    RCLCPP_INFO(logger_,
        "Publishing a %d x %d x %d touch_grid from site '%s' on '%s' (frame '%s') at %.1f Hz",
        nchannel_, height_, width_, site_name_.c_str(), topic_.c_str(), frame_id_.c_str(),
        rate());
    return true;
}

void TouchGridSensor::update(const mjData *mujoco_data, const rclcpp::Time &stamp) {
    // Float64MultiArray carries no header, so there is nowhere to put the
    // simulation time this sample was taken at.
    (void)stamp;

    // MuJoCo lays the grid out channel-major: [k*width*height + j*width + i].
    const mjtNum *source = mujoco_data->sensordata + sensor_adr_;
    for (int i = 0; i < sensor_dim_; i++) {
        data_[static_cast<size_t>(i)] = static_cast<double>(source[i]);
    }

    if (!realtime_publisher_) {
        return;
    }
    // trylock() keeps this non-blocking: update() runs with the simulation mutex
    // held, so waiting on the publishing thread here would stall mj_step.
    if (realtime_publisher_->trylock()) {
        realtime_publisher_->msg_.data = data_;
        realtime_publisher_->unlockAndPublish();
    }
}

bool TouchGridSensor::read_int_config(
        const mjModel *mujoco_model, int plugin_instance,
        const char *key, int minimum, int *out) {
    const char *value = mj_getPluginConfig(mujoco_model, plugin_instance, key);
    if (!value || !*value) {
        RCLCPP_ERROR(logger_, "touch_grid config '%s' is missing.", key);
        return false;
    }
    try {
        *out = std::stoi(value);
    } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "touch_grid config '%s' is not an integer: '%s'", key, value);
        return false;
    }
    if (*out < minimum) {
        RCLCPP_ERROR(logger_, "touch_grid config '%s' must be >= %d, got %d.", key, minimum, *out);
        return false;
    }
    return true;
}

bool TouchGridSensor::read_size_config(const mjModel *mujoco_model, int plugin_instance) {
    const char *value = mj_getPluginConfig(mujoco_model, plugin_instance, "size");
    if (!value || !*value) {
        RCLCPP_ERROR(logger_, "touch_grid config 'size' is missing.");
        return false;
    }
    std::istringstream stream{std::string(value)};
    if (!(stream >> width_ >> height_) || width_ < 1 || height_ < 1) {
        RCLCPP_ERROR(logger_,
            "touch_grid config 'size' must be two positive integers, got '%s'.", value);
        return false;
    }
    return true;
}

}  // namespace mujoco_ros2_control_examples

PLUGINLIB_EXPORT_CLASS(
    mujoco_ros2_control_examples::TouchGridSensor, mujoco_ros2_control::MujocoRos2PluginInterface)
