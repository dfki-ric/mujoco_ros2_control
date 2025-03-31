/**
 * @file wrench_sensor.cpp
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

#include "mujoco_ros2_sensors/wrench_sensor.hpp"
namespace mujoco_ros2_sensors {

    WrenchSensor::WrenchSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                           const WrenchSensorStruct &sensor, std::atomic<bool>* stop, double frequency) {
        this->nh_ = node;
        this->mujoco_data_ = data;
        this->sensor_ = sensor;

        this->publisher_ = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench", rclcpp::SystemDefaultsQoS());
        this->wrench_stamped_publisher_ = std::make_unique<WrenchStampedPublisher>(publisher_);
        wrench_stamped_publisher_->lock();
        wrench_stamped_publisher_->msg_.header.frame_id = sensor_.frame_id;
        wrench_stamped_publisher_->unlock();

        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&WrenchSensor::update, this));
    }

    void WrenchSensor::update() {
        if (wrench_stamped_publisher_->trylock()) {
            wrench_stamped_publisher_->msg_.header.stamp = nh_->now();
            if (sensor_.force) {
                wrench_stamped_publisher_->msg_.wrench.force.x = -mujoco_data_->sensordata[sensor_.force_sensor_adr];
                wrench_stamped_publisher_->msg_.wrench.force.y = -mujoco_data_->sensordata[sensor_.force_sensor_adr + 1];
                wrench_stamped_publisher_->msg_.wrench.force.z = -mujoco_data_->sensordata[sensor_.force_sensor_adr + 2];
            }

            if (sensor_.torque) {
                wrench_stamped_publisher_->msg_.wrench.torque.x = -mujoco_data_->sensordata[sensor_.torque_sensor_adr];
                wrench_stamped_publisher_->msg_.wrench.torque.y = -mujoco_data_->sensordata[sensor_.torque_sensor_adr + 1];
                wrench_stamped_publisher_->msg_.wrench.torque.z = -mujoco_data_->sensordata[sensor_.torque_sensor_adr + 2];
            }

            wrench_stamped_publisher_->unlockAndPublish();
        }
    }
}