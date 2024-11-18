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
        std::vector<WrenchSensorStruct> wrench_sensors;

        for (const auto &sensor : sensors_) {
            PoseSensorStruct pose_sensor;
            WrenchSensorStruct wrench_sensor;
            if (sensor.second.sensor_types[0] == mjSENS_FRAMEPOS || sensor.second.sensor_types[0] == mjSENS_FRAMEQUAT) {
                for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                    if (sensor.second.sensor_types[i] == mjSENS_FRAMEPOS) {
                        pose_sensor.body_name = sensor.first;
                        pose_sensor.position_sensor_adr = sensor.second.sensor_addresses[i];
                        pose_sensor.position = true;
                        if (pose_sensor.frame_id.empty()) {
                            pose_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                        } else if (pose_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                            RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create pose sensor, frames from position and orientation sensors doesn't match");
                            continue;
                        }
                    } else if (sensor.second.sensor_types[i] == mjSENS_FRAMEQUAT) {
                        pose_sensor.body_name = sensor.first;
                        pose_sensor.orientation_sensor_adr = sensor.second.sensor_addresses[i];
                        pose_sensor.orientation = true;
                        if (pose_sensor.frame_id.empty()) {
                            pose_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                        } else if (pose_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                            RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create pose sensor, frames from position and orientation sensors doesn't match");
                            continue;
                        }
                    }
                }
                pose_sensors.push_back(pose_sensor);
            }

            // TODO: Find out how to get the frame of a site
            if (sensor.second.sensor_types[0] == mjSENS_FORCE || sensor.second.sensor_types[0] == mjSENS_TORQUE) {
                for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                    if (sensor.second.sensor_types[i] == mjSENS_FORCE) {
                        wrench_sensor.body_name = sensor.first;
                        wrench_sensor.force_sensor_adr = sensor.second.sensor_addresses[i];
                        wrench_sensor.force = true;
                        //if (wrench_sensor.frame_id.empty()) {
                        //    wrench_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                        //} else if (wrench_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        //    RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create wrench sensor, frames from position and orientation sensors doesn't match");
                        //    continue;
                        //}
                    } else if (sensor.second.sensor_types[i] == mjSENS_TORQUE) {
                        wrench_sensor.body_name = sensor.first;
                        wrench_sensor.torque_sensor_adr = sensor.second.sensor_addresses[i];
                        wrench_sensor.torque = true;
                        //if (wrench_sensor.frame_id.empty()) {
                        //    wrench_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                        //} else if (wrench_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        //    RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create wrench sensor, frames from position and orientation sensors doesn't match");
                        //    continue;
                        //}
                    }
                }
                wrench_sensors.push_back(wrench_sensor);
            }
        }

        register_pose_sensors(pose_sensors);
        register_wrench_sensors(wrench_sensors);
    }

    MujocoRos2Sensors::~MujocoRos2Sensors() {
        for (auto &node : pose_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : pose_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : wrench_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : wrench_sensor_objs_) {
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

    void MujocoRos2Sensors::register_wrench_sensors(const std::vector<WrenchSensorStruct> &sensors) {
        wrench_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.force) {
                    value = "Force";
                } else if (sensor.torque) {
                    value = "Torque";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("wrench_sensor_registration"), "Wrench sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = wrench_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_wrench_sensor"));
            executor_->add_node(node);
            wrench_sensor_objs_.at(i).reset(new WrenchSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
        }
    }
}