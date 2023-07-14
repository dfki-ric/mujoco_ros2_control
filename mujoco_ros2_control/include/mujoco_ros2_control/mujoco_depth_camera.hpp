#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP


// @author: Gao Yinghao
// By Xiaomi Robotics Lab
// email: gaoyinghao@xiaomi.com

// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// OpenCV header
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL header
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "chrono"



// ROS header
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

namespace mujoco_sensors {
class MujocoDepthCamera {
public:
    MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data, int id, int res_x, int res_y,
                      double frequency, const std::string& name, bool *stop);
    ~MujocoDepthCamera();
    void update();

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int width_;
    int height_;

    mjModel* mujoco_model_ = nullptr;
    mjData* mujoco_data_ = nullptr;
    std::string name_;
    std::string body_name_;
    rclcpp::Time stamp_;

    GLFWwindow* window_;
    mjvCamera rgbd_camera_;
    mjrContext sensor_context_;
    mjvScene sensor_scene_;
    mjvOption sensor_option_;
    mjvPerturb sensor_perturb_;

    uchar* color_buffer_;    // color buffer
    float* depth_buffer_;  // depth buffer

    cv::Mat color_image_;
    cv::Mat depth_image_;

    // OpenGL render range
    double extent_;  // depth scale (m)
    double z_near_;  // near clipping plane depth
    double z_far_;   // far clipping plane depth
    // camera intrinsics
    double f_;   // focal length
    double cx_, cy_; // principal points

    double frequency_;

    /// @brief Linearize depth buffer and convert depth to depth in meters
    /// @param depth OpenGL depth buffer (nonlinearized)
    /// @return depth image in meters
    cv::Mat linearize_depth(const cv::Mat& depth) const;

    /// @brief This function sets the camera intrinsics. If the viewport size changes (e.g. you zoom the MuJoCo window), the camera intrinsics should be set again.
    /// @param model
    /// @param camera mujoco camera should be setup before using it
    /// @param viewport
    void set_camera_intrinsics(const mjModel* model, const mjvCamera camera, const mjrRect viewport);

    /// @brief Fetch OpenGL color buffer and depth buffer, and convert them to cv::Mats
    /// NOTE: Call release_buffer at the end of loop to avoid memory leak.
    /// @param model MuJoCo model
    /// @param viewport mjrRect Current viewport of MuJoCo
    /// @param context OpenGL context in MuJoCo
    void get_RGBD_buffer(const mjModel* model, const mjrRect viewport, const mjrContext* context);

    /// @brief free memory at the end of loop
    inline void release_buffer()
    {
        free(color_buffer_);
        free(depth_buffer_);
    }

    /// @brief Generate colorful pointcloud
    /// @return colorful pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> generate_color_point_cloud();

    pcl::PointCloud<pcl::PointXYZ> generate_point_cloud();

    void publish_point_cloud();
    void publish_image();
    void publish_camera_info();

};
}


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
