/**
 * @file wrench_sensor.hpp
 * @brief This file contains the implementation of wrench sensor.
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
#ifndef MUJOCO_ROS2_CONTROL_WRENCH_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_WRENCH_SENSOR_HPP
// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace mujoco_ros2_sensors {
    struct WrenchSensorStruct {
        std::string body_name;
        std::string frame_id;
        int force_sensor_adr;
        int torque_sensor_adr;
        bool force{false};
        bool torque{false};

        bool isValid() const {
            return force && torque;
        }
    };
    class WrenchSensor {
    public:

        WrenchSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                   const WrenchSensorStruct &sensor, std::atomic<bool>* stop, double frequency);
    private:
        void update();

        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the ROS 2 timer object used for scheduling periodic updates.


        // realtime_tools publisher for the clock message
        using WrenchStampedPublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
        using WrenchStampedPublisherPtr = std::unique_ptr<WrenchStampedPublisher>;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
        WrenchStampedPublisherPtr wrench_stamped_publisher_;

        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.

        WrenchSensorStruct sensor_;
    };
}
#endif //MUJOCO_ROS2_CONTROL_WRENCH_SENSOR_HPP
