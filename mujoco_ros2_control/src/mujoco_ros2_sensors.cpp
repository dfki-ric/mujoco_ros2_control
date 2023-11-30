#include <utility>

#include "mujoco_ros2_sensors/mujoco_ros2_sensors.hpp"

namespace mujoco_ros2_sensors {
    MujocoRos2Sensors::MujocoRos2Sensors(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, mjModel_ *model,
                                         mjData_ *data, std::map<std::string, Sensors> sensors) {
        this->executor_ = executor;
        this->mujoco_model_ = model;
        this->mujoco_data_ = data;
        this->sensors_ = std::move(sensors);

        std::vector<PoseSensorStruct> pose_sensors;

        for (const auto &sensor : sensors_) {
            PoseSensorStruct pose_sensor;
            pose_sensor.body_name = sensor.first;
            for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                if (sensor.second.sensor_types[i] == mjSENS_FRAMEPOS) {
                    pose_sensor.position_sensor_adr = sensor.second.sensor_addresses[i];
                    pose_sensor.position = true;
                    if (pose_sensor.frame_id.empty()) {
                        pose_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                    } else if (pose_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create pose sensor, frames from position and orientation sensors doesn't match");
                    }
                } else if (sensor.second.sensor_types[i] == mjSENS_FRAMEQUAT) {
                    pose_sensor.orientation_sensor_adr = sensor.second.sensor_addresses[i];
                    pose_sensor.orientation = true;
                    if (pose_sensor.frame_id.empty()) {
                        pose_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                    } else if (pose_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create pose sensor, frames from position and orientation sensors doesn't match");
                    }
                }
            }
            pose_sensors.push_back(pose_sensor);
        }

        register_pose_sensors(pose_sensors);
    }

    MujocoRos2Sensors::~MujocoRos2Sensors() {
        for (auto &node : pose_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : pose_sensor_objs_) {
            obj.reset();
        }
    }

    std::string MujocoRos2Sensors::get_frame_id(int sensor_id) {
        const auto &frame_id = mujoco_model_->sensor_refid[sensor_id];
        const auto &frame_type = mujoco_model_->sensor_reftype[sensor_id];
        return mj_id2name(mujoco_model_, frame_type, frame_id);
    }

    void MujocoRos2Sensors::register_pose_sensors(const std::vector<PoseSensorStruct> &sensors) {
        pose_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.position) {
                    value = "Position";
                } else if (sensor.orientation) {
                    value = "Orientation";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("pose_sensor_registration"), "Pose sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = pose_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_pose_sensor"));
            executor_->add_node(node);
            pose_sensor_objs_.at(i).reset(new PoseSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
        }
    }
}
