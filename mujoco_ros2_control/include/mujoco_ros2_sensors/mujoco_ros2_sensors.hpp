#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP

#include "chrono"

// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"
#include "mujoco_ros2_sensors/pose_sensor.hpp"

using namespace std::chrono_literals;

namespace mujoco_ros2_sensors {
    class MujocoRos2Sensors {
    public:
        struct Sensors {
            int obj_type;
            std::vector<std::string> sensor_names;
            std::vector<int> sensor_ids;
            std::vector<int> sensor_types;
            std::vector<int> sensor_addresses;
            std::vector<int> sensor_dimensions;
        };

        MujocoRos2Sensors(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, mjModel_ *model, mjData_ *data, std::map<std::string, Sensors> sensors);

        ~MujocoRos2Sensors();
    private:
        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_; ///< Shared pointer to the ROS 2 Node object used for communication and coordination.

        mjModel* mujoco_model_ = nullptr; ///< Pointer to the Mujoco model object used for rendering and simulation.
        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.
        rclcpp::Time stamp_; ///< ROS 2 timestamp representing the time when camera data was last updated.

        std::map<std::string, Sensors> sensors_;
        
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

        std::vector<PoseSensorStruct> pose_sensors_;
        std::vector<rclcpp::Node::SharedPtr> pose_sensor_nodes_; ///< Nodes for the cameras (one Node per camera)
        std::vector<std::shared_ptr<mujoco_ros2_sensors::PoseSensor>> pose_sensor_objs_; ///< Cameras Object vector

        void register_pose_sensors(const std::vector<PoseSensorStruct> &sensors);
        std::string get_frame_id(int sensor_id);
    };
}
#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP
