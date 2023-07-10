// https://github.com/gywhitel/mujoco_RGBD
#include "../include/mujoco_ros2_control/mujoco_depth_camera.hpp"

namespace mujoco_sensors {
    MujocoDepthCamera::MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data, int id, int res_x, int res_y, double frequency, const std::string& name) {
        nh_ = node;
        mujoco_model_ = model;
        mujoco_data_ = data;

        width_ = res_x;
        height_ = res_y;

        name_ = name;

        body_name_ = mj_id2name(mujoco_model_, mjOBJ_BODY, mujoco_model_->cam_bodyid[id]);

        glfwInit();
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(width_, height_, name_.c_str(), NULL, NULL);
        // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        // setup camera
        rgbd_camera_.type = mjCAMERA_FIXED;
        rgbd_camera_.fixedcamid = id;

        mjv_defaultOption(&sensor_option_);
        mjv_defaultScene(&sensor_scene_);
        mjr_defaultContext(&sensor_context_);
        mjv_defaultPerturb(&sensor_perturb_);

        // create scene and context
        mjv_makeScene(mujoco_model_, &sensor_scene_, 1000);
        mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);

        mjr_setBuffer(mjFB_OFFSCREEN, &sensor_context_);

        //color_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        camera_info_publisher_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>("~/color/camera_info", 10);
        color_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("~/color/image_raw", 10);
        depth_image_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>("~/depth/image_rect_raw", 10);
        pointcloud_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/color/points", 10);
        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0s/frequency), std::bind(&MujocoDepthCamera::update, this));
    }

    MujocoDepthCamera::~MujocoDepthCamera() {
        update_thread_.join();
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
    }

    void MujocoDepthCamera::update() {
        // get framebuffer viewport
        glfwMakeContextCurrent(window_);
        mjrRect viewport = {0,0,0,0};
        set_camera_intrinsics(mujoco_model_, rgbd_camera_, viewport);
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL, &rgbd_camera_, mjCAT_ALL, &sensor_scene_);
        mjr_render(viewport, &sensor_scene_, &sensor_context_);

        get_RGBD_buffer(mujoco_model_, viewport, &sensor_context_);
        stamp_ = nh_->now();

        // Swap OpenGL buffers
        glfwSwapBuffers(window_);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
        // Do not forget to release buffer to avoid memory leak
        release_buffer();
        publish_image();
        publish_point_cloud();
        publish_camera_info();
    }

    cv::Mat MujocoDepthCamera::linearize_depth(const cv::Mat& depth) const {
        cv::Mat depth_img(depth.size(), CV_32F, cv::Scalar(0));

        for (int i = 0; i < depth_img.rows; i++) {
            auto* raw_depth_ptr = depth.ptr<float>(i);
            auto* m_depth_ptr = depth_img.ptr<float>(i);

            for (uint j = 0; j < depth_img.cols; j++) {
                m_depth_ptr[j] = z_near_ * z_far_ * extent_ / (z_far_ - raw_depth_ptr[j] * (z_far_ - z_near_));
            }
        }
        return depth_img;
    }

    void MujocoDepthCamera::set_camera_intrinsics(const mjModel* model, const mjvCamera camera, const mjrRect viewport) {
        // vertical FOV
        double fovy = model->cam_fovy[camera.fixedcamid] / 180 * M_PI / 2;

        // focal length, fx = fy
        f_ = (float) viewport.height / 2.0 / tan(fovy);

        // principal points
        cx_ = viewport.width / 2;
        cy_ = viewport.height / 2;
    }

    void MujocoDepthCamera::get_RGBD_buffer(const mjModel* model, const mjrRect viewport, const mjrContext* context) {
        // Use preallocated buffer to fetch color buffer and depth buffer in OpenGL
        color_buffer_ = (uchar*) malloc(viewport.height*viewport.width * 3);
        depth_buffer_ = (float*) malloc(viewport.height*viewport.width * 4);
        mjr_readPixels(color_buffer_, depth_buffer_, viewport, context);

        extent_ = model->stat.extent;
        z_near_ = model->vis.map.znear;
        z_far_ = model->vis.map.zfar;

        cv::Size img_size(viewport.width, viewport.height);
        cv::Mat rgb(img_size, CV_8UC3, color_buffer_);
        cv::flip(rgb, rgb, 0);
        rgb.copyTo(color_image_);
        cv::imshow("Image", rgb);
        cv::waitKey(1);

        cv::Mat depth(img_size, CV_32FC1, depth_buffer_);
        cv::flip(depth, depth, 0);
        cv::Mat depth_img_m = linearize_depth(depth);
        depth_img_m.copyTo(depth_image_);
        cv::imshow("Depth", depth_image_);
        cv::waitKey(1);
    }

    pcl::PointCloud<pcl::PointXYZ> MujocoDepthCamera::generate_pointcloud() {
        using namespace pcl;
        PointCloud<PointXYZ> cloud;

        for (int i = 0; i < depth_image_.rows; i++) {
            for (int j = 0; j < depth_image_.cols; j++) {
                double depth = *(depth_image_.ptr<float>(i,j));
                // filter far points
                if (depth < z_far_) {
                    PointXYZ point;
                    point.x = static_cast<float>(double(j - cx_) * depth / f_);
                    point.y = static_cast<float>(double(i - cy_) * depth / f_);
                    point.z = static_cast<float>(depth);

                    cloud.push_back(point);
                }
            }
        }
        return cloud;
    }


    pcl::PointCloud<pcl::PointXYZRGB> MujocoDepthCamera::generate_color_pointcloud() {
        using namespace pcl;
        // color image and depth image should have the same size and should be aligned
        assert(color_image_.size() == depth_image_.size());

        PointCloud<PointXYZRGB> rgb_cloud;

        for (int i = 0; i < color_image_.rows; i++) {
            for (int j = 0; j < color_image_.cols; j++) {
                double depth = *(depth_image_.ptr<float>(i,j));
                // filter far points
                if (depth < z_far_) {
                    PointXYZRGB rgb_3d_point;
                    rgb_3d_point.x = static_cast<float>(double(j - cx_) * depth / f_);
                    rgb_3d_point.y = static_cast<float>(double(i - cy_) * depth / f_);
                    rgb_3d_point.z = static_cast<float>(depth);

                    const uchar* bgr_ptr = color_image_.ptr<uchar>(i,j);
                    rgb_3d_point.r = bgr_ptr[0];
                    rgb_3d_point.g = bgr_ptr[1];
                    rgb_3d_point.b = bgr_ptr[2];
                    rgb_cloud.push_back(rgb_3d_point);
                }
            }
        }
        return rgb_cloud;
    }

    void MujocoDepthCamera::publish_camera_info() {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header.stamp = stamp_;
        camera_info.header.frame_id = body_name_;
        camera_info.height = height_;
        camera_info.width = width_;
        camera_info_publisher_->publish(camera_info);
    }

    void MujocoDepthCamera::publish_image() {
        cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->image = color_image_;
        cv_ptr->encoding = "8UC3";
        sensor_msgs::msg::Image out_image;
        cv_ptr->toImageMsg(out_image);
        //out_image.header.stamp = stamp_;
        //out_image.header.frame_id = body_name_;
        color_image_publisher_->publish(out_image);

//        cv_ptr->image = depth_image_;
//        cv_ptr->encoding = "32FC1";
//        cv_ptr->toImageMsg(out_image);
//        out_image.header.stamp = stamp_;
//        out_image.header.frame_id = body_name_;
//        depth_image_publisher_->publish(out_image);
    }



    void MujocoDepthCamera::publish_point_cloud() {
        sensor_msgs::msg::PointCloud2 out_cloud;
        pcl::toROSMsg(generate_color_pointcloud(), out_cloud);
        out_cloud.header.stamp = stamp_;
        out_cloud.header.frame_id = body_name_;
        pointcloud_publisher_->publish(out_cloud);
    }
}
