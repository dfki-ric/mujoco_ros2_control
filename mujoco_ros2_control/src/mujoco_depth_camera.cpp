/**
 * @file mujoco_depth_camera.cpp
 *
 * @brief This file contains the implementation of the Mujoco DepthCamera.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license GNU General Public License, version 3 (GPL-3.0)
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * This file is governed by the GNU General Public License, version 3 (GPL-3.0).
 * The GPL-3.0 is a copyleft license that allows users to use, modify, and distribute software
 * while ensuring that these freedoms are passed on to subsequent users. It requires that any
 * derivative works or modifications of the software be licensed under the GPL-3.0 as well.
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 * This interaction with the OpenGL camera is based on the work by Gao Yinghao, Xiaomi Robotics Lab.
 * Email: gaoyinghao@xiaomi.com
 *
 * The original code can be found in the following repository:
 * https://github.com/gywhitel/mujoco_RGBD
 */

#include "../include/mujoco_ros2_control/mujoco_depth_camera.hpp"

namespace mujoco_sensors {
    MujocoDepthCamera::MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data, int id,
                                         int res_x, int res_y, double frequency, const std::string& name,
                                         std::atomic<bool>* stop) {
        nh_ = node;
        mujoco_model_ = model;
        mujoco_data_ = data;

        width_ = res_x;
        height_ = res_y;

        name_ = name;
        body_name_ = mj_id2name(mujoco_model_, mjOBJ_BODY, mujoco_model_->cam_bodyid[id]);
        frequency_ = frequency;
        stop_ = stop;

        // init GLFW
        if (!glfwInit()) {
            RCLCPP_ERROR(nh_->get_logger(), "Could not initialize GLFW");
        }

        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(width_, height_, name_.c_str(), NULL, NULL);
        glfwSetWindowAttrib(window_, GLFW_RESIZABLE, GLFW_FALSE);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        // setup camera
        rgbd_camera_.type = mjCAMERA_FIXED;
        rgbd_camera_.fixedcamid = id;

        mjr_defaultContext(&sensor_context_);
        mjv_defaultOption(&sensor_option_);
        mjv_defaultScene(&sensor_scene_);

        // create scene and context
        mjv_makeScene(mujoco_model_, &sensor_scene_, 2000);
        mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);

        mjr_setBuffer(mjFB_OFFSCREEN, &sensor_context_);

        color_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + name_ + "/color/camera_info", 10);
        depth_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + name_ + "/depth/camera_info", 10);
        color_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + name_ + "/color/image_raw", 10);
        depth_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + name_ + "/depth/image_rect_raw", 10);
        pointcloud_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/" + name_ + "/depth/color/points", 10);
    }

    MujocoDepthCamera::~MujocoDepthCamera() {
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
    }

    void MujocoDepthCamera::update() {
        mjtNum last_update = mujoco_data_->time;
        while(rclcpp::ok() && !stop_->load()) {
            if (mujoco_data_->time - last_update >= 1.0 / frequency_) {

                last_update = mujoco_data_->time;
                glfwMakeContextCurrent(window_);

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
                set_camera_intrinsics(viewport);
                mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL, &rgbd_camera_, mjCAT_ALL,
                                &sensor_scene_);

                // update scene and render
                mjr_render(viewport, &sensor_scene_, &sensor_context_);
                get_RGBD_buffer(viewport);
                stamp_ = nh_->now();

                // Swap OpenGL buffers
                glfwSwapBuffers(window_);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();

                publish_images();
                publish_point_cloud();
                publish_camera_info();
                release_buffer();
            }
        }
    }

    cv::Mat MujocoDepthCamera::linearize_depth(const cv::Mat& depth) const {
        cv::Mat depth_img(depth.size(), CV_32FC1, cv::Scalar(0));

        for (int i = 0; i < depth_img.rows; i++) {
            auto* raw_depth_ptr = depth.ptr<float>(i);
            auto* m_depth_ptr = depth_img.ptr<float>(i);

            for (uint j = 0; j < depth_img.cols; j++) {
                m_depth_ptr[j] = z_near_ * z_far_ * extent_ / (z_far_ - raw_depth_ptr[j] * (z_far_ - z_near_));
            }
        }
        return depth_img;
    }

    void MujocoDepthCamera::set_camera_intrinsics(const mjrRect viewport) {
        // vertical FOV
        double fovy = mujoco_model_->cam_fovy[rgbd_camera_.fixedcamid] / 180 * M_PI;

        // focal length, fx = fy
        f_ = (static_cast<double>(viewport.height) / 2.0) / tan(fovy/2.0);

        // principal points
        cx_ = viewport.width / 2.0;
        cy_ = viewport.height / 2.0;
    }

    void MujocoDepthCamera::get_RGBD_buffer(const mjrRect viewport) {
        // Use preallocated buffer to fetch color buffer and depth buffer in OpenGL
        color_buffer_ = (uchar*) malloc(viewport.height*viewport.width * 3);
        depth_buffer_ = (float*) malloc(viewport.height*viewport.width * 4);
        mjr_readPixels(color_buffer_, depth_buffer_, viewport, &sensor_context_);

        extent_ = mujoco_model_->stat.extent;
        z_near_ = mujoco_model_->vis.map.znear;
        z_far_ = mujoco_model_->vis.map.zfar;

        cv::Size img_size(viewport.width, viewport.height);
        cv::Mat bgr(img_size, CV_8UC3, color_buffer_);
        cv::flip(bgr, bgr, 0);
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        rgb.copyTo(color_image_);


        cv::Mat depth(img_size, CV_32FC1, depth_buffer_);
        cv::flip(depth, depth, 0);
        cv::Mat depth_img_m = linearize_depth(depth);
        depth_img_m.copyTo(depth_image_);
    }

    pcl::PointCloud<pcl::PointXYZRGB> MujocoDepthCamera::generate_color_point_cloud() {
        using namespace pcl;
        // color image and depth image should have the same size and should be aligned
        assert(color_image_.size() == depth_image_.size());

        PointCloud<PointXYZRGB> rgb_cloud;
        rgb_cloud.height = color_image_.rows;
        rgb_cloud.width = color_image_.cols;
        rgb_cloud.is_dense = false;
        rgb_cloud.points.resize(rgb_cloud.height * rgb_cloud.width);
        rgb_cloud.header.frame_id = body_name_;

        for (int i = 0; i < color_image_.rows; i++) {
            for (int j = 0; j < color_image_.cols; j++) {
                double depth = *(depth_image_.ptr<float>(i,j));
                // filter far points
                if (depth < z_far_) {
                    rgb_cloud.at(j, i).x = static_cast<float>(double(j - cx_) * depth / f_);
                    rgb_cloud.at(j, i).y = static_cast<float>(double(i - cy_) * depth / f_);
                    rgb_cloud.at(j, i).z = static_cast<float>(depth);

                    const uchar* bgr_ptr = color_image_.ptr<uchar>(i,j);
                    rgb_cloud.at(j, i).r = bgr_ptr[2];
                    rgb_cloud.at(j, i).g = bgr_ptr[1];
                    rgb_cloud.at(j, i).b = bgr_ptr[0];
                }
            }
        }

        Eigen::Matrix4f trans;
        trans << 0.0, 0.0, 1.0, 0.0,
                 -1.0, 0.0, 0.0, 0.0,
                 0.0, -1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

        pcl::PointCloud<pcl::PointXYZRGB> transformed;
        pcl::transformPointCloud(rgb_cloud, transformed, trans);

        return transformed;
    }

    void MujocoDepthCamera::publish_camera_info() {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header.stamp = stamp_;
        camera_info.header.frame_id = body_name_;
        camera_info.height = height_;
        camera_info.width = width_;
        camera_info.distortion_model = "plumb_bob";
        camera_info.k = {f_, 0.0, cx_,
                         0.0, f_, cy_,
                         0.0, 0.0, 1.0};

        camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        camera_info.p = {f_, 0.0, cx_, 0,
                         0.0, f_, cy_, 0,
                         0.0, 0.0, 1.0, 0.0};

        color_camera_info_publisher_->publish(camera_info);
        depth_camera_info_publisher_->publish(camera_info);
    }

    void MujocoDepthCamera::publish_images() {
        cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->image = color_image_;
        cv_ptr->encoding = "8UC3";
        sensor_msgs::msg::Image out_image;
        cv_ptr->toImageMsg(out_image);
        out_image.header.stamp = stamp_;
        out_image.header.frame_id = body_name_;
        color_image_publisher_->publish(out_image);

        cv_ptr->image = depth_image_;
        cv_ptr->encoding = "32FC1";
        cv_ptr->toImageMsg(out_image);
        out_image.header.stamp = stamp_;
        out_image.header.frame_id = body_name_;
        depth_image_publisher_->publish(out_image);
    }

    void MujocoDepthCamera::publish_point_cloud() {
        sensor_msgs::msg::PointCloud2 out_cloud;
        pcl::toROSMsg(generate_color_point_cloud(), out_cloud);
        out_cloud.header.stamp = stamp_;
        pointcloud_publisher_->publish(out_cloud);
    }
}
