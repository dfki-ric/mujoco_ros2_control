#ifndef MUJOCO_ROS2_CONTROL_WRENCH_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_WRENCH_SENSOR_HPP
// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"

#include "realtime_tools/realtime_publisher.h"
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
