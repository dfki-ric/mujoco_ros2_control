// https://github.com/gywhitel/mujoco_RGBD
#include "../include/mujoco_ros2_control/mujoco_depth_camera.hpp"

namespace mujoco_sensors {
    MujocoDepthCamera::MujocoDepthCamera(mjModel_ *model, mjData_ *data, int id, int res_x, int res_y, double frequency, const std::string& name) : Node(name){
        mujoco_model_ = model;
        mujoco_data_ = data;

        name_ = name;

        body_id_ = mujoco_model_->cam_bodyid[id];

        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        window_ = glfwCreateWindow(res_x, res_y, name_.c_str(), NULL, NULL);
        // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        // setup camera
        rgbd_camera_.type = mjCAMERA_FIXED;
        rgbd_camera_.fixedcamid = id;

        mjv_defaultOption(&sensor_option_);
        mjv_defaultScene(&sensor_scene_);
        mjr_defaultContext(&sensor_context_);

        // create scene and context
        mjv_makeScene(mujoco_model_, &sensor_scene_, 1000);
        mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);

        publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);
        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0s/frequency), std::bind(&MujocoDepthCamera::update, this));
    }

    MujocoDepthCamera::~MujocoDepthCamera() {
        RCLCPP_INFO(rclcpp::get_logger("sensor"), "TEST 1111");
        update_thread_.join();
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
        RCLCPP_INFO(rclcpp::get_logger("sensor"), "TEST 2222");
    }

    void MujocoDepthCamera::update() {
        // get framebuffer viewport
        mjrRect viewport = {0,0,0,0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        set_camera_intrinsics(mujoco_model_, rgbd_camera_, viewport);

        // update scene and render
        mjv_updateScene(mujoco_model_, mujoco_data_, &sensor_option_, NULL, &rgbd_camera_, mjCAT_ALL, &sensor_scene_);
        mjr_render(viewport, &sensor_scene_, &sensor_context_);

        get_RGBD_buffer(mujoco_model_, viewport, &sensor_context_);

        mtx_.lock();
        *color_cloud_ = generate_color_pointcloud();
        sensor_msgs::msg::PointCloud2 out_cloud;
        pcl::toROSMsg(*color_cloud_, out_cloud);
        publisher_->publish(out_cloud);
        mtx_.unlock();

        // Swap OpenGL buffers
        glfwSwapBuffers(window_);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // Do not forget to release buffer to avoid memory leak
        release_buffer();
    }

    cv::Mat MujocoDepthCamera::linearize_depth(const cv::Mat& depth) const {
        cv::Mat depth_img(depth.size(), CV_32F, cv::Scalar(0));

        for (int i = 0; i < depth_img.rows; i++) {
            auto* raw_depth_ptr = depth.ptr<float>(i);
            auto* m_depth_ptr = depth_img.ptr<float>(i);

            for (uint j = 0; j < depth_img.cols; j++) {
                m_depth_ptr[j] = z_near * z_far * extent / (z_far - raw_depth_ptr[j] * (z_far - z_near));
            }
        }
        return depth_img;
    }

    void MujocoDepthCamera::set_camera_intrinsics(const mjModel* model, const mjvCamera camera, const mjrRect viewport) {
        // vertical FOV
        double fovy = model->cam_fovy[camera.fixedcamid] / 180 * M_PI / 2;

        // focal length, fx = fy
        f = (float) viewport.height / 2.0 / tan(fovy);

        // principal points
        cx = viewport.width / 2;
        cy = viewport.height / 2;
    }

    void MujocoDepthCamera::get_RGBD_buffer(const mjModel* model, const mjrRect viewport, const mjrContext* context) {
        // Use preallocated buffer to fetch color buffer and depth buffer in OpenGL
        color_buffer = (uchar*) malloc(viewport.height*viewport.width * 3);
        depth_buffer = (float*) malloc(viewport.height*viewport.width * 4);
        mjr_readPixels(color_buffer, depth_buffer, viewport, context);

        extent = model->stat.extent;
        z_near = model->vis.map.znear;
        z_far = model->vis.map.zfar;

        cv::Size img_size(viewport.width, viewport.height);
        cv::Mat rgb(img_size, CV_8UC3, color_buffer);
        cv::flip(rgb, rgb, 0);
        rgb.copyTo(color_image);

        cv::Mat depth(img_size, CV_32F, depth_buffer);
        cv::flip(depth, depth, 0);
        cv::Mat depth_img_m = linearize_depth(depth);
        depth_img_m.copyTo(depth_image);
    }

    pcl::PointCloud<pcl::PointXYZ> MujocoDepthCamera::generate_pointcloud() {
        using namespace pcl;
        PointCloud<PointXYZ> cloud;

        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                double depth = *(depth_image.ptr<float>(i,j));
                // filter far points
                if (depth < z_far) {
                    PointXYZ point;
                    point.x = static_cast<float>(double(j - cx) * depth / f);
                    point.y = static_cast<float>(double(i - cy) * depth / f);
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
        assert(color_image.size() == depth_image.size());

        PointCloud<PointXYZRGB> rgb_cloud;
        rgb_cloud.header.frame_id = name_;

        for (int i = 0; i < color_image.rows; i++) {
            for (int j = 0; j < color_image.cols; j++) {
                double depth = *(depth_image.ptr<float>(i,j));
                // filter far points
                if (depth < z_far) {
                    PointXYZRGB rgb_3d_point;
                    rgb_3d_point.x = static_cast<float>(double(j - cx) * depth / f);
                    rgb_3d_point.y = static_cast<float>(double(i - cy) * depth / f);
                    rgb_3d_point.z = static_cast<float>(depth);

                    const uchar* bgr_ptr = color_image.ptr<uchar>(i,j);
                    rgb_3d_point.r = bgr_ptr[0];
                    rgb_3d_point.g = bgr_ptr[1];
                    rgb_3d_point.b = bgr_ptr[2];
                    rgb_cloud.push_back(rgb_3d_point);
                }
            }
        }
        return rgb_cloud;
    }
}
