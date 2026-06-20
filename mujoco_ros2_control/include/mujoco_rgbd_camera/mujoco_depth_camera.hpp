/**
 * @file mujoco_depth_camera.hpp
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

#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP

#include "chrono"

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
#include <pcl/common/transforms.h>

// ROS header
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "mujoco_rgbd_camera_parameters.hpp"

using namespace std::chrono_literals;

namespace mujoco_rgbd_camera {
/**
 * @class MujocoDepthCamera
 * @brief Represents a depth camera in a Mujoco simulation environment.
 *
 * The MujocoDepthCamera class provides functionality to capture camera data, including color images, depth images,
 * and point clouds, from a Mujoco simulation environment. It uses the Mujoco physics engine for rendering and simulation,
 * and ROS 2 for communication and coordination with other nodes.
 *
 * The class allows for continuous capturing and publishing of camera data based on a specified frequency. It supports
 * setting camera intrinsics, retrieving RGB-D buffers from OpenGL, generating color point clouds, and publishing camera
 * information, color images, depth images, and point cloud data. It provides methods for updating the camera data and
 * releasing resources when the camera is no longer needed.
 *
 * The class utilizes several dependencies, including rclcpp for ROS 2 integration, GLFW for window management, and PCL and
 * OpenCV for point cloud and image processing. It provides various member variables and methods to handle camera-related
 * data and operations.
 */
class MujocoDepthCamera {
public:
    /**
     * @brief How the camera is mounted in the MuJoCo model.
     *
     * - FixedCamera: a MuJoCo `<camera>` element. Pose and vertical FOV are taken
     *   from the model camera (`mjCAMERA_FIXED`).
     * - Site: a MuJoCo `<site>` whose frame is interpreted as a REP-103 optical
     *   frame (+Z forward, +X right, +Y down). The scene camera is positioned from
     *   the site every frame (`mjCAMERA_USER`) and the vertical FOV comes from the
     *   `fovy` ROS parameter.
     */
    enum class Mount { FixedCamera, Site };

    /**
     * @brief Constructor for MujocoDepthCamera class.
     *
     * @param node Pointer to the ROS 2 Node object.
     * @param model Pointer to the Mujoco model object.
     * @param data Pointer to the Mujoco data object.
     * @param name Name of the camera (used for the node and topic namespace).
     * @param stop Pointer to the atomic boolean flag indicating whether the camera should stop or continue.
     * @param mount How the camera is mounted (a `<camera>` element or a `<site>`).
     * @param optical_id Mount id for the color/optical stream: a camera id when
     *        @p mount is Mount::FixedCamera, otherwise the MuJoCo site id of the
     *        optical site (or < 0 if the camera has no color stream).
     * @param depth_id Mount id for the depth stream: a camera id when @p mount is
     *        Mount::FixedCamera, otherwise the MuJoCo site id of the depth site
     *        (or < 0 if the camera has no depth stream). When @p optical_id ==
     *        @p depth_id the two streams share a single render pass.
     *
     * @post Initializes the MujocoDepthCamera object with the provided parameters and sets up the required ROS 2 publishers.
     *       Initializes the GLFW library and creates a hidden GLFW window.
     *       Sets up the Mujoco camera properties.
     *       Creates and initializes the Mujoco scene and context for rendering.
     *       Creates ROS 2 publishers for camera information, color image, depth image, and point cloud data.
     */
    MujocoDepthCamera(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                      const std::string& name, std::atomic<bool>* stop,
                      Mount mount, int optical_id, int depth_id);

    /**
     * @brief Destroys the `MujocoDepthCamera` object.
     *
     * This destructor cleans up the OpenGL Ressources.
     * It is responsible for releasing any dynamically allocated memory or performing any necessary cleanup operations.
     * It is automatically called when the object goes out of scope or is explicitly deleted.
     */
    ~MujocoDepthCamera();

    /**
     * @brief Updates the MujocoDepthCamera by continuously capturing camera data and publishing it.
     *
     * The method runs in a loop until the stop flag is set to true or ROS 2 is no longer okay.
     * It checks the time elapsed since the last update and, if enough time has passed according to the camera frequency,
     * performs the following steps:
     *   - Makes the GLFW window's context current.
     *   - Retrieves the framebuffer viewport size and sets camera intrinsics.
     *   - Updates the Mujoco scene and renders it using the provided context and camera settings.
     *   - Gets the RGB-D buffer from the Mujoco model and viewport.
     *   - Retrieves the current timestamp.
     *   - Swaps OpenGL buffers.
     *   - Processes pending GUI events and GLFW callbacks.
     *   - Publishes the captured color image, depth image, point cloud, and camera information.
     *   - Releases the buffer to avoid memory leaks.
     *
     * @post The camera data is continuously captured and published until the stop flag is set to true or ROS 2 is no longer okay.
     */
    void update();

private:

    // Parameters from ROS2 using generate_parameter_library
    std::shared_ptr<ParamListener> param_listener_;
    mujoco_rgbd_camera::Params params_;

    std::atomic<bool>* stop_; ///< Pointer to an atomic boolean flag indicating whether the camera should stop or continue.

    rclcpp::Node::SharedPtr nh_; ///< Shared pointer to the ROS 2 Node object used for communication and coordination.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_publisher_; ///< Shared pointer to the ROS 2 publisher for color images.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_; ///< Shared pointer to the ROS 2 publisher for depth images.
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_publisher_; ///< Shared pointer to the ROS 2 publisher for color camera information.
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_publisher_; ///< Shared pointer to the ROS 2 publisher for depth camera information.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_; ///< Shared pointer to the ROS 2 publisher for point cloud data.

    int width_; ///< Width of the camera image in pixels.
    int height_; ///< Height of the camera image in pixels.
    double frequency_; ///< Frame rate of the camera in Hz.

    mjModel* mujoco_model_ = nullptr; ///< Pointer to the Mujoco model object used for rendering and simulation.
    mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.
    std::string name_; ///< Name of the camera.
    std::string body_name_; ///< Resolved default frame_id (frame_id override, else the parent body name).
    std::string color_frame_; ///< header.frame_id for color image/info (the optical site's render frame / parent body).
    std::string depth_frame_; ///< header.frame_id for depth image/info and cloud (the depth site's render frame / parent body).

    Mount mount_ = Mount::FixedCamera; ///< How the camera is mounted (a <camera> element or a <site>).
    int optical_id_ = -1; ///< Camera id (FixedCamera) or optical site id (Site) for the color stream; < 0 = no color.
    int depth_id_ = -1; ///< Camera id (FixedCamera) or depth site id (Site) for the depth stream; < 0 = no depth.
    bool have_color_ = false; ///< Whether a color stream is available (optical mount resolved).
    bool have_depth_ = false; ///< Whether a depth stream is available (depth mount resolved).
    double fovy_ = 45.0; ///< Vertical field of view [deg], used for site-based cameras.
    double fovx_ = 0.0; ///< Horizontal field of view [deg] for site-based cameras; <= 0 means derive from fovy and the aspect ratio.

    int render_width_ = 0;  ///< GL framebuffer width used for rendering (may differ from width_ when fovx is set independently).
    int render_height_ = 0; ///< GL framebuffer height used for rendering.
    rclcpp::Time stamp_; ///< ROS 2 timestamp representing the time when camera data was last updated.

    GLFWwindow* window_; ///< Pointer to the GLFW window used for rendering.
    mjvCamera rgbd_camera_{}; ///< Mujoco visualization camera object representing the RGB-D camera.
    mjrContext sensor_context_{}; ///< Mujoco render context for the sensor camera.
    mjvScene sensor_scene_{}; ///< Mujoco visualization scene for rendering.
    mjvOption sensor_option_{}; ///< Mujoco visualization options for the sensor camera.

    cv::Mat color_image_; ///< Published color image (rendered from the optical site).
    cv::Mat depth_image_; ///< Published depth image (rendered from the depth site).
    cv::Mat depth_color_image_; ///< Color rendered from the depth site, used to color the point cloud so RGB and depth stay aligned.

    // OpenGL render range
    double extent_{};  ///< Depth scale (m) for the OpenGL render range.
    double z_near_{};  ///< Near clipping plane depth for the OpenGL render range.
    double z_far_{};   ///< Far clipping plane depth for the OpenGL render range.

    // camera intrinsics
    double fx_{};  ///< Horizontal focal length [px] (fx). Equals fy_ for square-pixel cameras.
    double fy_{};  ///< Vertical focal length [px] (fy).
    double cx_{}, cy_{}; ///< Principal points of the camera.


    /**
     * @brief Linearizes the depth values in the input depth image.
     *
     * Creates a new depth image with the same size as the input depth image,
     * initialized with all zeros and of type CV_32FC1 (32-bit floating-point, single channel).
     *
     * Iterates over each row of the depth image and retrieves the raw depth values.
     * Calculates the linearized depth value for each pixel using the provided formula:
     *     linearized_depth = z_near_ * z_far_ * extent_ / (z_far_ - raw_depth * (z_far_ - z_near_))
     * where raw_depth is the depth value at the corresponding pixel in the input depth image.
     * Assigns the linearized depth values to the corresponding pixels in the new depth image.
     *
     * @param depth The input depth image to be linearized.
     * @return The linearized depth image.
     */
    cv::Mat linearize_depth(const cv::Mat& depth) const;

    /**
     * @brief Sets the camera intrinsic parameters based on the given model, camera, and viewport.
     *
     * Calculates and sets the camera intrinsic parameters, including the focal length (f_) and principal points (cx_, cy_),
     * based on the vertical field of view (fovy) obtained from the model and camera.
     *
     * The focal length is calculated as half the viewport height divided by the tangent of half the vertical field of view (fovy/2).
     * The principal points are set to the center of the viewport.
     *
     * @param viewport The rendering viewport specifying the width and height of the camera image.
     */
    void set_camera_intrinsics(mjrRect viewport);

    /**
     * @brief Aims the two scene GL cameras at the given site (site-based cameras).
     *
     * The abstract camera is `mjCAMERA_USER`, so the two `mjvGLCamera` entries in
     * the scene must be set explicitly. The site frame is interpreted as a REP-103
     * optical frame (+Z forward, +X right, +Y down): the GL view direction is the
     * site's +Z axis and the GL up vector is the site's -Y axis. A symmetric vertical
     * frustum is built from `fovy_`; the horizontal FOV follows from the GL framebuffer
     * aspect ratio.
     *
     * @param site_id The MuJoCo site id to aim the scene cameras at.
     */
    void aim_at_site(int site_id);

    /**
     * @brief Reads the current GL framebuffer into the requested OpenCV images.
     *
     * Reads the color and/or depth buffers from OpenGL for the given viewport. The
     * color buffer is flipped to top-left origin and converted to RGB; the depth
     * buffer is flipped and linearized to metric depth. When the render viewport
     * differs from the published resolution (site cameras with an independent fovx),
     * the result is resampled to width_ x height_ (linear for color, nearest for
     * depth so foreground/background depths are never blended). Pass nullptr for a
     * buffer that should not be read.
     *
     * @param viewport      The rendering viewport (the GL framebuffer size).
     * @param color_out     Destination for the RGB image, or nullptr to skip color.
     * @param depth_out     Destination for the metric depth image, or nullptr to skip depth.
     */
    void read_buffers(mjrRect viewport, cv::Mat* color_out, cv::Mat* depth_out);

    /**
     * @brief Generates a color point cloud from the color and depth images.
     *
     * Iterates over each pixel in the color and depth images.
     * Retrieves the depth value at the corresponding pixel in the depth image.
     * Filters out far points by checking if the depth value is less than the predefined z_far_ threshold.
     * Calculates the 3D coordinates (x, y, z) of the point based on the depth value, camera intrinsics (focal length, principal point),
     * and the pixel's position relative to the principal point.
     * Assigns the RGB values from the color image to the corresponding point in the color point cloud.
     *
     * Applies a transformation matrix to the color point cloud to align it with the desired coordinate system.
     * The transformation matrix transposes the coordinates to have the view in the X direction.
     *
     * Returns the transformed organized color point cloud.
     */
    pcl::PointCloud<pcl::PointXYZRGB> generate_color_point_cloud();

    /**
     * @brief Publishes camera information for the color and depth images.
     *
     * Creates and populates a `sensor_msgs::msg::CameraInfo` object with relevant camera parameters,
     * such as timestamp, frame ID, image resolution, distortion model, intrinsic matrix, distortion coefficients,
     * and projection matrix.
     *
     * Publishes the camera information for both the color and depth images using the respective ROS 2 publishers.
     *
     * @post Camera information for the color and depth images is published.
     */
    void publish_point_cloud();

    /**
     * @brief Publishes color and depth images.
     *
     * Creates a `cv_bridge::CvImagePtr` object and sets its image data and encoding type
     * to the color image and depth image respectively.
     *
     * Converts the `cv_ptr` to a `sensor_msgs::msg::Image` object using `toImageMsg()`
     * and populates the header with the timestamp and frame ID.
     *
     * Publishes the color and depth images using the respective ROS 2 publishers.
     *
     * @post Color and depth images are published.
     */
    void publish_images();

    /**
     * @brief Publishes a point cloud generated from the color image.
     *
     * Creates a `sensor_msgs::msg::PointCloud2` object named `out_cloud`.
     *
     * Generates a color point cloud using the `generate_color_point_cloud()` method and converts it to a ROS message format using `pcl::toROSMsg()`.
     *
     * Populates the header of `out_cloud` with the timestamp and sets it to the frame ID of the camera body.
     *
     * Publishes the point cloud using the ROS 2 publisher.
     *
     * @post The generated point cloud is published.
     */
    void publish_camera_info();

};
}


#endif //MUJOCO_ROS2_CONTROL_MUJOCO_DEPTH_CAMERA_HPP
