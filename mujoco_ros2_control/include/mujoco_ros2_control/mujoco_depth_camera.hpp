#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP


// @author: Gao Yinghao
// By Xiaomi Robotics Lab
// email: gaoyinghao@xiaomi.com

// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"

// OpenCV header
#include <opencv2/opencv.hpp>

// PCL header
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

namespace mujoco_sensors {
class MujocoDepthCamera {
public:
    MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data, int id, int res_x, int res_y, double frequency, const std::string& name);
    ~MujocoDepthCamera();
    void update();

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    mjModel* mujoco_model_ = nullptr;
    mjData* mujoco_data_ = nullptr;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud_;
    std::string name_;
    size_t body_id_;

    GLFWwindow* window_;
    mjvCamera rgbd_camera_;
    mjrContext sensor_context_;
    mjvScene sensor_scene_;
    mjvOption sensor_option_;
    mjvPerturb sensor_perturb_;

    uchar* color_buffer;    // color buffer
    float* depth_buffer;  // depth buffer

    cv::Mat color_image;
    cv::Mat depth_image;

    // OpenGL render range
    double extent;  // depth scale (m)
    double z_near;  // near clipping plane depth
    double z_far;   // far clipping plane depth
    // camera intrinsics
    double f;   // focal length
    int cx, cy; // principal points

    std::thread update_thread_;


    /// @brief Linearize depth buffer and convert depth to depth in meters
    /// @param depth OpenGL depth buffer (nonlinearized)
    /// @return depth image in meters
    cv::Mat linearize_depth(const cv::Mat& depth) const;

    std::mutex mtx_;

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
        free(color_buffer);
        free(depth_buffer);
    }

    /// @brief Generate monochrome pointcloud
    /// @return monochrome pointcloud
    pcl::PointCloud<pcl::PointXYZ> generate_pointcloud();

    /// @brief Generate colorful pointcloud
    /// @return colorful pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> generate_color_pointcloud();

    inline cv::Mat get_color_image()
    {
        return color_image;
    }

    inline cv::Mat get_depth_image()
    {
        return depth_image;
    }
};
}


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
