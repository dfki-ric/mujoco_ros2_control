#include "mujoco_ros2_sensors/pose_sensor.hpp"
namespace mujoco_ros2_sensors {

    PoseSensor::PoseSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                           const PoseSensorStruct &sensor, std::atomic<bool>* stop, double frequency) {
        this->nh_ = node;
        this->mujoco_data_ = data;
        this->sensor_ = sensor;

        this->publisher_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>("~/position", rclcpp::SystemDefaultsQoS());
        this->pose_stamped_publisher_ = std::make_unique<PoseStampedPublisher>(publisher_);
        pose_stamped_publisher_->lock();
        pose_stamped_publisher_->msg_.header.frame_id = sensor_.frame_id;
        pose_stamped_publisher_->unlock();

        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&PoseSensor::update, this));
    }

    void PoseSensor::update() {
        if (pose_stamped_publisher_->trylock()) {
            pose_stamped_publisher_->msg_.header.stamp.sec = std::floor(mujoco_data_->time);
            pose_stamped_publisher_->msg_.header.stamp.nanosec = std::floor((mujoco_data_->time-std::floor(mujoco_data_->time))*1e9);
            //pose_stamped_publisher_->msg_.header.frame_id = sensor_.frame_id;
            if (sensor_.position) {
                pose_stamped_publisher_->msg_.pose.position.x = mujoco_data_->sensordata[sensor_.position_sensor_adr];
                pose_stamped_publisher_->msg_.pose.position.y = mujoco_data_->sensordata[sensor_.position_sensor_adr + 1];
                pose_stamped_publisher_->msg_.pose.position.z = mujoco_data_->sensordata[sensor_.position_sensor_adr + 2];
            }

            if (sensor_.orientation) {
                pose_stamped_publisher_->msg_.pose.orientation.w = mujoco_data_->sensordata[sensor_.orientation_sensor_adr];
                pose_stamped_publisher_->msg_.pose.orientation.x = mujoco_data_->sensordata[sensor_.orientation_sensor_adr + 1];
                pose_stamped_publisher_->msg_.pose.orientation.y = mujoco_data_->sensordata[sensor_.orientation_sensor_adr + 2];
                pose_stamped_publisher_->msg_.pose.orientation.z = mujoco_data_->sensordata[sensor_.orientation_sensor_adr + 3];
            }

            pose_stamped_publisher_->unlockAndPublish();
        }
    }
}