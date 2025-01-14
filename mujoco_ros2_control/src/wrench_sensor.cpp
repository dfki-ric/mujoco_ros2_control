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