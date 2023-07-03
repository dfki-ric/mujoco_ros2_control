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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>

namespace mujoco_sensors {
class MujocoDepthCamera {
private:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_;
    std::string name_;

    GLFWwindow* window_;
    mjvCamera rgbd_camera_;
    mjrContext sensor_context_;
    mjvScene sensor_scene_;
    mjvOption sensor_option_;

    uchar* color_buffer{};    // color buffer
    float* depth_buffer{};  // depth buffer

    cv::Mat color_image;
    cv::Mat depth_image;

    // OpenGL render range
    double extent{};  // depth scale (m)
    double z_near{};  // near clipping plane depth
    double z_far{};   // far clipping plane depth
    // camera intrinsics
    double f{};   // focal length
    int cx{}, cy{}; // principal points


    /// @brief Linearize depth buffer and convert depth to depth in meters
    /// @param depth OpenGL depth buffer (nonlinearized)
    /// @return depth image in meters
    cv::Mat linearize_depth(const cv::Mat& depth) const;

    std::mutex mtx_;



public:
    MujocoDepthCamera(mjModel* mujoco_model, mjData *mujoco_data, int id);

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

    void update();
};
}

pcl::visualization::PCLVisualizer::Ptr color_cloud_visual (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

pcl::visualization::PCLVisualizer::Ptr cloud_visual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
