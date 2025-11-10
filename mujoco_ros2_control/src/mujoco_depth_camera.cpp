/**
 * @file mujoco_depth_camera.cpp
 *
 * @brief This file contains the implementation of the Mujoco DepthCamera.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This interaction with the OpenGL camera is based on the work by Gao Yinghao, Xiaomi Robotics Lab.
 * Email: gaoyinghao@xiaomi.com
 *
 * The original code can be found in the following repository:
 * https://github.com/gywhitel/mujoco_RGBD
 */

#include "mujoco_rgbd_camera/mujoco_depth_camera.hpp"

namespace mujoco_rgbd_camera {
    MujocoDepthCamera::MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data, int id,
                                         const std::string& name, std::atomic<bool>* stop) {
        nh_ = node;

        // set up the parameter listener
        param_listener_ = std::make_shared<ParamListener>(nh_);
        param_listener_->refresh_dynamic_parameters();

        params_ = param_listener_->get_params();

        mujoco_model_ = model;
        mujoco_data_ = data;

        width_ = params_.width;
        height_ = params_.height;
        frequency_ = params_.frequency;


        name_ = name;
        body_name_ = mj_id2name(mujoco_model_, mjOBJ_BODY, mujoco_model_->cam_bodyid[id]);

        stop_ = stop;

        // init GLFW
        if (!glfwInit()) {
            RCLCPP_ERROR(nh_->get_logger(), "Could not initialize GLFW");
        }

        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        window_ = glfwCreateWindow(width_, height_, name_.c_str(), NULL, NULL);
        glfwSetWindowAttrib(window_, GLFW_RESIZABLE, GLFW_FALSE);
        auto context = glfwGetCurrentContext();
        glfwMakeContextCurrent(window_);

        // Set camera parameters
        rgbd_camera_.type = mjCAMERA_FIXED;
        rgbd_camera_.fixedcamid = id; // Set the ID of the fixed camera you want to use

        mjr_defaultContext(&sensor_context_);
        mjv_defaultOption(&sensor_option_);
        mjv_defaultScene(&sensor_scene_);

        // create scene and context
        mjv_makeScene(mujoco_model_, &sensor_scene_, 2000);
        mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);

        mjr_setBuffer(mjFB_WINDOW, &sensor_context_);

        if (params_.color_image) {
            color_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + name_ + "/color/camera_info", 10);
            color_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + name_ + "/color/image_raw", 10);
        }
        if (params_.depth_image) {
            depth_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + name_ + "/depth/camera_info", 10);
            depth_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + name_ + "/depth/image_rect_raw", 10);
        }
        if (params_.point_cloud) {
            pointcloud_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/" + name_ + "/depth/points", 10);
        }
        glfwMakeContextCurrent(context);

    }

    MujocoDepthCamera::~MujocoDepthCamera() {
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
    }

    void MujocoDepthCamera::update() {
        mjtNum last_update = mujoco_data_->time;
        while(rclcpp::ok() && !stop_->load()) {
            // update dynamic parameters
            if (mujoco_data_->time - last_update >= 1.0 / frequency_) {
                last_update = mujoco_data_->time;
                auto context = glfwGetCurrentContext();
                glfwMakeContextCurrent(window_);

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
                set_camera_intrinsics(viewport);
                mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL, &rgbd_camera_, mjCAT_ALL, &sensor_scene_);

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
                glfwMakeContextCurrent(context);
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
        if (!color_buffer_ || !depth_buffer_) {
            RCLCPP_ERROR(nh_->get_logger(), "Failed to allocate color or depth buffer!");
        }

        extent_ = mujoco_model_->stat.extent;
        z_near_ = mujoco_model_->vis.map.znear;
        z_far_ = mujoco_model_->vis.map.zfar;

        cv::Size img_size(viewport.width, viewport.height);
        cv::Mat bgr(img_size, CV_8UC3, color_buffer_);
        cv::flip(bgr, bgr, -1);
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        rgb.copyTo(color_image_);


        cv::Mat depth(img_size, CV_32FC1, depth_buffer_);
        cv::flip(depth, depth, -1);
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

        pcl::PointXYZRGB* point_ptr = rgb_cloud.data();
        for (int i = 0; i < color_image_.rows; i++) {
            for (int j = 0; j < color_image_.cols; j++) {
                double depth = *(depth_image_.ptr<float>(i,j));
                // filter far points
                if (depth < z_far_) {
                    point_ptr->x = static_cast<float>(depth);
                    point_ptr->y = static_cast<float>(double(j - cx_) * depth / f_);
                    point_ptr->z = -static_cast<float>(double(i - cy_) * depth / f_);

                    const uchar* bgr_ptr = color_image_.ptr<uchar>(i,j);
                    point_ptr->r = bgr_ptr[2];
                    point_ptr->g = bgr_ptr[1];
                    point_ptr->b = bgr_ptr[0];
                }
                point_ptr++;
            }
        }

        return rgb_cloud;
    }

    void MujocoDepthCamera::publish_camera_info() {
        if (!params_.color_image && !params_.depth_image) {
            return;
        }
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

        if (params_.color_image) {
            color_camera_info_publisher_->publish(camera_info);
        }

        if (params_.depth_image) {
            depth_camera_info_publisher_->publish(camera_info);
        }
    }

    void MujocoDepthCamera::publish_images() {
        if (!params_.color_image && !params_.depth_image) {
            return;
        }

        cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
        sensor_msgs::msg::Image out_image;

        if (params_.color_image) {
            cv_ptr->image = color_image_;
            cv_ptr->encoding = "8UC3";
            cv_ptr->toImageMsg(out_image);
            out_image.header.stamp = stamp_;
            out_image.header.frame_id = body_name_;
            color_image_publisher_->publish(out_image);
        }

        if (params_.depth_image) {
            cv_ptr->image = depth_image_;
            cv_ptr->encoding = "32FC1";
            cv_ptr->toImageMsg(out_image);
            out_image.header.stamp = stamp_;
            out_image.header.frame_id = body_name_;
            depth_image_publisher_->publish(out_image);
        }

    }

    void MujocoDepthCamera::publish_point_cloud() {
        if (params_.point_cloud) {
            sensor_msgs::msg::PointCloud2 out_cloud;
            pcl::toROSMsg(generate_color_point_cloud(), out_cloud);
            out_cloud.header.stamp = stamp_;
            pointcloud_publisher_->publish(out_cloud);
        }
    }
}
