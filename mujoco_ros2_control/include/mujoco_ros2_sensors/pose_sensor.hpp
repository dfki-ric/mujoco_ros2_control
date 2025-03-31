/**
 * @file pose_sensor.hpp
 * @brief This file contains the implementation of pose sensor.
 *
 * @author Adrian Danzglock
 * @date 2023
 *

 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2023, DFKI GmbH
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
#ifndef MUJOCO_ROS2_CONTROL_POSE_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_POSE_SENSOR_HPP
// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TODO: Make it possible to choose if want to use tf or pose stamped
namespace mujoco_ros2_sensors {
    struct PoseSensorStruct {
        std::string body_name;
        std::string frame_id;
        int position_sensor_adr;
        int orientation_sensor_adr;
        bool position{false};
        bool orientation{false};

        bool isValid() const {
            return position && orientation;
        }
    };
    class PoseSensor {
    public:

        PoseSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                   const PoseSensorStruct &sensor, std::atomic<bool>* stop, double frequency);
    private:
        void update();

        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the ROS 2 timer object used for scheduling periodic updates.

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped t_;

        // fallback to pose publisher when position or orientation is missing
        using PoseStampedPublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>;
        using PoseStampedPublisherPtr = std::unique_ptr<PoseStampedPublisher>;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        PoseStampedPublisherPtr pose_stamped_publisher_;

        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.

        PoseSensorStruct sensor_;
    };
}
#endif //MUJOCO_ROS2_CONTROL_POSE_SENSOR_HPP
