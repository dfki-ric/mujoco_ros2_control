/**
 * @file pose_sensor.hpp
 * @brief This file contains the implementation of imu sensor.
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
#ifndef MUJOCO_ROS2_CONTROL_IMU_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_IMU_SENSOR_HPP
// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

// TODO: Make it possible to choose if want to use tf or pose stamped
namespace mujoco_ros2_sensors {
    struct ImuSensorStruct {
        std::string body_name;
        std::string frame_id;
        int gyro_sensor_adr;
        int accel_sensor_adr;
        bool gyro{false};
        bool accel{false};

        bool isValid() const {
            return gyro && accel;
        }
    };
    class ImuSensor {
    public:
        ImuSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                   const ImuSensorStruct &sensor, std::atomic<bool>* stop, double frequency);
    private:
        void update();

        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the ROS 2 timer object used for scheduling periodic updates.

        // fallback to pose publisher when position or orientation is missing
        using ImuPublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>;
        using ImuPublisherPtr = std::unique_ptr<ImuPublisher>;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        ImuPublisherPtr imu_publisher_;

        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.

        ImuSensorStruct sensor_;
    };
}
#endif //MUJOCO_ROS2_CONTROL_IMU_SENSOR_HPP
