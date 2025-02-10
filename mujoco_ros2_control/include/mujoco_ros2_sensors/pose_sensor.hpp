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
