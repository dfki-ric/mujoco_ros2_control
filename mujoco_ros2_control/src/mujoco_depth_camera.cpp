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

#include <algorithm>
#include <cmath>
#include <vector>

namespace mujoco_rgbd_camera {
    MujocoDepthCamera::MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                                         const std::string& name, std::atomic<bool>* stop, Mount mount,
                                         int optical_id, int depth_id) {
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
        fovy_ = params_.fovy;
        fovx_ = params_.fovx;

        name_ = name;
        mount_ = mount;
        optical_id_ = optical_id;
        depth_id_ = depth_id;
        have_color_ = optical_id_ >= 0;
        have_depth_ = depth_id_ >= 0;

        // The GL framebuffer is normally the published resolution. For a site
        // camera with an independent fovx (> 0), MuJoCo can only render square
        // angular pixels, so we render at the aspect implied by (fovx, fovy) and
        // resample to width_ x height_ later. That gives the requested fovx and
        // fovy with independent fx/fy (e.g. to match a calibrated RealSense).
        render_width_ = width_;
        render_height_ = height_;
        if (mount_ == Mount::Site && fovx_ > 0.0) {
            const double aspect =
                std::tan(fovx_ / 360.0 * M_PI) / std::tan(fovy_ / 360.0 * M_PI);
            render_height_ = height_;
            render_width_ = std::max(1, static_cast<int>(std::lround(height_ * aspect)));
        }

        // The published header.frame_id is the render frame: the body each mount is
        // attached to (the <camera>'s body, or the site's parent body). Color and
        // depth can therefore live in different frames when fed by different sites.
        auto body_name_of = [&](int mount_id) -> std::string {
            const int body_id = (mount_ == Mount::Site)
                ? mujoco_model_->site_bodyid[mount_id]
                : mujoco_model_->cam_bodyid[mount_id];
            const char *n = mj_id2name(mujoco_model_, mjOBJ_BODY, body_id);
            return n ? std::string(n) : std::string();
        };
        if (have_color_) color_frame_ = body_name_of(optical_id_);
        if (have_depth_) depth_frame_ = body_name_of(depth_id_);
        body_name_ = have_depth_ ? depth_frame_ : color_frame_;

        // Clip planes are stored in the model as fractions of the model extent;
        // cache them so the GL frustum (site cameras) and the depth linearization
        // use exactly the same near/far values the fixed-camera path would.
        extent_ = mujoco_model_->stat.extent;
        z_near_ = mujoco_model_->vis.map.znear;
        z_far_ = mujoco_model_->vis.map.zfar;

        stop_ = stop;

        // init GLFW
        if (!glfwInit()) {
            RCLCPP_ERROR(nh_->get_logger(), "Could not initialize GLFW");
        }

        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(render_width_, render_height_, name_.c_str(), NULL, NULL);
        glfwSetWindowAttrib(window_, GLFW_RESIZABLE, GLFW_FALSE);
        auto context = glfwGetCurrentContext();
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(0);

        // Set camera parameters
        if (mount_ == Mount::Site) {
            // The scene cameras are positioned from the site every frame.
            rgbd_camera_.type = mjCAMERA_USER;
        } else {
            rgbd_camera_.type = mjCAMERA_FIXED;
            rgbd_camera_.fixedcamid = depth_id_; // <camera> mounts pass the camera id as both ids
        }

        mjr_defaultContext(&sensor_context_);
        mjv_defaultOption(&sensor_option_);
        mjv_defaultScene(&sensor_scene_);

        // create scene and context
        mjv_makeScene(mujoco_model_, &sensor_scene_, 2000);
        mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);

        mjr_setBuffer(mjFB_WINDOW, &sensor_context_);

        // Topic namespace defaults to the node name but can be overridden. A stream
        // is only published if its mount (optical/depth site) is present.
        const std::string ns = params_.topic_namespace.empty() ? name_ : params_.topic_namespace;
        if (params_.color_image && have_color_) {
            color_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + ns + "/color/camera_info", 10);
            color_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + ns + "/color/image_raw", 10);
        }
        if (params_.depth_image && have_depth_) {
            depth_camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + ns + "/depth/camera_info", 10);
            depth_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("/" + ns + "/depth/image_rect_raw", 10);
        }
        if (params_.point_cloud && have_depth_) {
            pointcloud_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/" + ns + "/depth/points", 10);
        }
        glfwMakeContextCurrent(context);

    }

    MujocoDepthCamera::~MujocoDepthCamera() {
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
    }

    void MujocoDepthCamera::update() {
        mjtNum last_update = mujoco_data_->time;
        const bool want_color = have_color_ && params_.color_image;
        const bool want_depth = have_depth_ && (params_.depth_image || params_.point_cloud);
        // A single shared render pass covers both streams for a <camera> mount or a
        // single mjCam_ site (optical and depth are the same view). Separate optical
        // and depth sites render in two passes (true parallax).
        const bool single_pass = (mount_ != Mount::Site) || (optical_id_ == depth_id_);
        while(rclcpp::ok() && !stop_->load()) {
            // resync if the simulation was reset (sim time jumped backwards)
            if (mujoco_data_->time < last_update) {
                last_update = mujoco_data_->time;
            }
            // update dynamic parameters
            if (mujoco_data_->time - last_update >= 1.0 / frequency_) {
                last_update += 1.0 / frequency_;
                if (mujoco_data_->time - last_update >= 1.0 / frequency_) {
                    last_update = mujoco_data_->time;
                }
                auto context = glfwGetCurrentContext();
                glfwMakeContextCurrent(window_);

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
                set_camera_intrinsics(viewport);

                if (single_pass) {
                    // One render gives color + depth from the same viewpoint; only
                    // the enabled buffers are read back.
                    if (mount_ == Mount::Site) {
                        aim_at_site(have_depth_ ? depth_id_ : optical_id_);
                    }
                    mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL,
                                    &rgbd_camera_, mjCAT_ALL, &sensor_scene_);
                    mjr_render(viewport, &sensor_scene_, &sensor_context_);
                    const bool read_color = want_color || params_.point_cloud;
                    read_buffers(viewport,
                                 read_color ? &color_image_ : nullptr,
                                 want_depth ? &depth_image_ : nullptr);
                    depth_color_image_ = color_image_;  // same viewpoint -> aligned
                } else {
                    // Depth pass: depth image (+ a color render at the depth site to
                    // color the cloud, only when a cloud is published). Skipped
                    // entirely if neither depth nor cloud is enabled.
                    if (want_depth) {
                        aim_at_site(depth_id_);
                        mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL,
                                        &rgbd_camera_, mjCAT_ALL, &sensor_scene_);
                        mjr_render(viewport, &sensor_scene_, &sensor_context_);
                        read_buffers(viewport,
                                     params_.point_cloud ? &depth_color_image_ : nullptr,
                                     &depth_image_);
                    }
                    // Optical pass: published color image. Skipped if color is off.
                    if (want_color) {
                        aim_at_site(optical_id_);
                        mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL,
                                        &rgbd_camera_, mjCAT_ALL, &sensor_scene_);
                        mjr_render(viewport, &sensor_scene_, &sensor_context_);
                        read_buffers(viewport, &color_image_, nullptr);
                    }
                }
                stamp_ = nh_->now();

                // Swap OpenGL buffers
                glfwSwapBuffers(window_);

                publish_images();
                publish_point_cloud();
                publish_camera_info();
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
        // Intrinsics describe the published image (width_ x height_), not the GL
        // framebuffer (which may differ when fovx is set independently).
        // vertical FOV: from the MJCF camera for <camera> mounts, from fovy for sites.
        const double fovy_deg = (mount_ == Mount::Site)
            ? fovy_
            : mujoco_model_->cam_fovy[depth_id_];
        const double fovy = fovy_deg / 180.0 * M_PI;
        fy_ = (static_cast<double>(height_) / 2.0) / tan(fovy / 2.0);

        // horizontal FOV: an independent fovx (site cameras only) gives fx != fy;
        // otherwise pixels are square (fx == fy).
        if (mount_ == Mount::Site && fovx_ > 0.0) {
            const double fovx = fovx_ / 180.0 * M_PI;
            fx_ = (static_cast<double>(width_) / 2.0) / tan(fovx / 2.0);
        } else {
            fx_ = fy_;
        }

        // principal points
        cx_ = width_ / 2.0;
        cy_ = height_ / 2.0;
        (void)viewport;
    }

    void MujocoDepthCamera::aim_at_site(int site_id) {
        // Site world pose: position and 3x3 rotation (row-major).
        const mjtNum *spos = mujoco_data_->site_xpos + 3 * site_id;
        const mjtNum *smat = mujoco_data_->site_xmat + 9 * site_id;

        // The site frame is a REP-103 optical frame (+Z forward, +X right, +Y down).
        // MuJoCo's GL camera looks along +forward with +up as image-up, so the view
        // direction is the site +Z axis and up is the site -Y axis (both in world).
        const float forward[3] = {
            static_cast<float>(smat[2]), static_cast<float>(smat[5]), static_cast<float>(smat[8])
        };
        const float up[3] = {
            static_cast<float>(-smat[1]), static_cast<float>(-smat[4]), static_cast<float>(-smat[7])
        };

        // Symmetric vertical frustum from fovy; the horizontal extent follows from
        // the GL framebuffer aspect ratio (render_width_/render_height_), which is
        // chosen so the rendered horizontal FOV equals fovx. Near/far use the same
        // metric clip planes the fixed-camera path renders with.
        const double fovy = fovy_ / 180.0 * M_PI;
        const double znear_m = z_near_ * extent_;
        const double zfar_m = z_far_ * extent_;
        const double half_v = std::tan(fovy / 2.0) * znear_m;

        for (int eye = 0; eye < 2; ++eye) {
            mjvGLCamera &gc = sensor_scene_.camera[eye];
            gc.pos[0] = static_cast<float>(spos[0]);
            gc.pos[1] = static_cast<float>(spos[1]);
            gc.pos[2] = static_cast<float>(spos[2]);
            gc.forward[0] = forward[0];
            gc.forward[1] = forward[1];
            gc.forward[2] = forward[2];
            gc.up[0] = up[0];
            gc.up[1] = up[1];
            gc.up[2] = up[2];
            gc.frustum_center = 0.0f;
            gc.frustum_bottom = static_cast<float>(-half_v);
            gc.frustum_top    = static_cast<float>( half_v);
            gc.frustum_near   = static_cast<float>(znear_m);
            gc.frustum_far    = static_cast<float>(zfar_m);
        }
    }

    void MujocoDepthCamera::read_buffers(const mjrRect viewport, cv::Mat* color_out, cv::Mat* depth_out) {
        if (!color_out && !depth_out) {
            return;
        }

        const size_t pixels = static_cast<size_t>(viewport.height) * viewport.width;
        std::vector<uchar> color_buffer(color_out ? pixels * 3 : 0);
        std::vector<float> depth_buffer(depth_out ? pixels : 0);
        mjr_readPixels(color_out ? color_buffer.data() : nullptr,
                       depth_out ? depth_buffer.data() : nullptr,
                       viewport, &sensor_context_);

        // The GL framebuffer may be rendered at a different aspect than the
        // published resolution (site cameras with an independent fovx). Resample
        // to width_ x height_: linear for color, nearest for depth so we never
        // blend foreground and background depths into spurious values.
        const cv::Size img_size(viewport.width, viewport.height);
        const bool resample = (viewport.width != width_ || viewport.height != height_);
        const cv::Size out_size(width_, height_);

        if (color_out) {
            cv::Mat bgr(img_size, CV_8UC3, color_buffer.data());
            // OpenGL framebuffer has origin at bottom-left; flip vertically (code 0)
            // to get standard top-left image origin. Flipping both axes (code -1)
            // would also mirror the image horizontally.
            cv::flip(bgr, bgr, 0);
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            if (resample) {
                cv::resize(rgb, *color_out, out_size, 0, 0, cv::INTER_LINEAR);
            } else {
                rgb.copyTo(*color_out);
            }
        }

        if (depth_out) {
            cv::Mat depth(img_size, CV_32FC1, depth_buffer.data());
            cv::flip(depth, depth, 0);
            cv::Mat depth_img_m = linearize_depth(depth);
            if (resample) {
                cv::resize(depth_img_m, *depth_out, out_size, 0, 0, cv::INTER_NEAREST);
            } else {
                depth_img_m.copyTo(*depth_out);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> MujocoDepthCamera::generate_color_point_cloud() {
        using namespace pcl;
        // The cloud is colored from depth_color_image_ (a color render at the depth
        // site), which is aligned with depth_image_ even when color and depth use
        // separate optical sites.
        const bool has_color = depth_color_image_.size() == depth_image_.size();

        PointCloud<PointXYZRGB> rgb_cloud;
        rgb_cloud.height = depth_image_.rows;
        rgb_cloud.width = depth_image_.cols;
        rgb_cloud.is_dense = false;
        rgb_cloud.points.resize(rgb_cloud.height * rgb_cloud.width);
        rgb_cloud.header.frame_id = depth_frame_;

        pcl::PointXYZRGB* point_ptr = rgb_cloud.data();
        for (int i = 0; i < depth_image_.rows; i++) {
            for (int j = 0; j < depth_image_.cols; j++) {
                double depth = *(depth_image_.ptr<float>(i,j));
                // filter far points
                if (depth < z_far_) {
                    // REP-103 optical frame: X right, Y down, Z forward
                    point_ptr->x = static_cast<float>(double(j - cx_) * depth / fx_);
                    point_ptr->y = static_cast<float>(double(i - cy_) * depth / fy_);
                    point_ptr->z = static_cast<float>(depth);

                    if (has_color) {
                        // color_image_/depth_color_image_ hold BGR-ordered bytes
                        // (the GL RGB buffer wrapped as BGR + cvtColor swap), so
                        // map index [2,1,0] -> RGB.
                        const uchar* bgr_ptr = depth_color_image_.ptr<uchar>(i,j);
                        point_ptr->r = bgr_ptr[2];
                        point_ptr->g = bgr_ptr[1];
                        point_ptr->b = bgr_ptr[0];
                    }
                }
                point_ptr++;
            }
        }

        return rgb_cloud;
    }

    void MujocoDepthCamera::publish_camera_info() {
        if (!color_camera_info_publisher_ && !depth_camera_info_publisher_) {
            return;
        }
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header.stamp = stamp_;
        camera_info.height = height_;
        camera_info.width = width_;
        camera_info.distortion_model = "plumb_bob";
        camera_info.k = {fx_, 0.0, cx_,
                         0.0, fy_, cy_,
                         0.0, 0.0, 1.0};

        camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        camera_info.p = {fx_, 0.0, cx_, 0,
                         0.0, fy_, cy_, 0,
                         0.0, 0.0, 1.0, 0.0};

        if (color_camera_info_publisher_) {
            camera_info.header.frame_id = color_frame_;
            color_camera_info_publisher_->publish(camera_info);
        }

        if (depth_camera_info_publisher_) {
            camera_info.header.frame_id = depth_frame_;
            depth_camera_info_publisher_->publish(camera_info);
        }
    }

    void MujocoDepthCamera::publish_images() {
        if (!color_image_publisher_ && !depth_image_publisher_) {
            return;
        }

        cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
        sensor_msgs::msg::Image out_image;

        if (color_image_publisher_) {
            cv_ptr->image = color_image_;
            cv_ptr->encoding = "8UC3";
            cv_ptr->toImageMsg(out_image);
            out_image.header.stamp = stamp_;
            out_image.header.frame_id = color_frame_;
            color_image_publisher_->publish(out_image);
        }

        if (depth_image_publisher_) {
            cv_ptr->image = depth_image_;
            cv_ptr->encoding = "32FC1";
            cv_ptr->toImageMsg(out_image);
            out_image.header.stamp = stamp_;
            out_image.header.frame_id = depth_frame_;
            depth_image_publisher_->publish(out_image);
        }

    }

    void MujocoDepthCamera::publish_point_cloud() {
        if (pointcloud_publisher_) {
            sensor_msgs::msg::PointCloud2 out_cloud;
            pcl::toROSMsg(generate_color_point_cloud(), out_cloud);
            out_cloud.header.stamp = stamp_;
            pointcloud_publisher_->publish(out_cloud);
        }
    }
}