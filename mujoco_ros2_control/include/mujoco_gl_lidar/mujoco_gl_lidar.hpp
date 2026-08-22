/**
 * @file mujoco_gl_lidar.hpp
 *
 * @brief OpenGL-based lidar for MuJoCo. Renders a depth image at the lidar
 * site's pose, then samples per-beam ranges from the depth buffer. Reuses the
 * EGL / mjvScene / mjrContext pattern from MujocoDepthCamera.
 *
 * @author Adrian Danzglock
 * @date 2026
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2026, DFKI GmbH
 */

#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_GL_LIDAR_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_GL_LIDAR_HPP

#include <array>
#include <atomic>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "mujoco/mujoco.h"
#include <EGL/egl.h>
#include "GL/gl.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <mujoco_ros2_control/mujoco_gl_lidar_parameters.hpp>

namespace mujoco_gl_lidar {

class MujocoGLLidar {
public:
    /**
     * @param node       Per-lidar ROS node (created by the plugin, use_sim_time on).
     * @param model      MuJoCo model.
     * @param data       Live MuJoCo data. Snapshotted under sim_mutex once per
     *                   scan; the snapshot is what gets rendered/sampled, so
     *                   all slices and beams within a scan are time-consistent.
     * @param sim_mutex  Mutex held by the sim thread around mj_step. Locked
     *                   briefly here for the per-scan mj_copyData.
     * @param site_id    Index of the MuJoCo site the lidar is mounted on.
     *                   Site +X is the 0-rad scan direction; rotation around +Z.
     * @param name       Unique name (used for topic and frame_id).
     * @param stop       Plugin-owned stop flag; the worker exits when set.
     */
    MujocoGLLidar(rclcpp::Node::SharedPtr &node,
                  mjModel_ *model,
                  mjData_ *data,
                  std::mutex *sim_mutex,
                  int site_id,
                  const std::string &name,
                  std::atomic<bool> *stop);

    ~MujocoGLLidar();

    void update();

private:
    void render_depth();
    bool sample_beam(size_t beam_index, double &range,
                     double *cos_incidence = nullptr);
    void publish_scan(const rclcpp::Time &stamp);
    void publish_cloud(const rclcpp::Time &stamp);

    std::shared_ptr<mujoco_gl_lidar::ParamListener> param_listener_;
    mujoco_gl_lidar::Params params_;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    mjModel *mujoco_model_ = nullptr;
    mjData  *mujoco_data_  = nullptr;       // live data, copied from under sim_mutex_
    mjData  *snapshot_data_ = nullptr;      // per-scan deep copy used for rendering
    std::mutex *sim_mutex_ = nullptr;       // held by sim thread around mj_step
    std::atomic<bool> *stop_ = nullptr;

    int site_id_;
    int parent_body_id_;
    std::string name_;
    std::string frame_id_;

    // Site pose relative to its parent body (constant: sites are rigidly
    // attached).
    mjtNum site_pos_in_body_[3]{};
    mjtNum site_mat_in_body_[9]{};

    enum class OutputKind { Scan, Cloud };
    OutputKind output_kind_;
    double frequency_;
    double range_min_, range_max_;
    double h_min_, h_max_;
    int    h_samples_;
    double v_min_, v_max_;
    int    v_samples_;
    double noise_stddev_;

    // Render config
    int render_width_;       // pixels per slice (height is shared)
    int render_height_;
    int n_slices_ = 1;
    double depth_threshold_; // max depth difference for bilinear interpolation
    double slice_width_;          // unpadded yaw width per slice (rad)
    double slice_render_h_fov_;   // slice_width_ + 2*fov_padding
    double v_min_padded_;         // vertical_min_angle - fov_padding (rad)
    double v_max_padded_;         // vertical_max_angle + fov_padding (rad)
    double v_fov_;                // v_max_padded - v_min_padded
    double v_center_;             // (v_min_padded + v_max_padded) / 2 — camera pitch tilt
    double cos_v_center_;         // cached trig for sampling
    double sin_v_center_;
    double f_;                    // focal length (px), same per slice
    double cx_, cy_;              // principal point (px), same per slice
    double extent_;          // model.stat.extent
    double znear_frac_;      // model.vis.map.znear (relative to extent)
    double zfar_frac_;       // model.vis.map.zfar  (relative to extent)
    double znear_, zfar_;    // absolute clip distances [m]

    // Pre-computed beam directions in the lidar's local (site) frame
    std::vector<std::array<double, 3>> beam_dirs_local_;

    // Per-beam sampling info (precomputed)
    struct BeamSample {
        int slice;
        double dx, dy, dz;  // unit direction in slice-local frame
    };
    std::vector<BeamSample> beam_samples_;

    // EGL state for offscreen rendering
    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    EGLContext egl_context_ = EGL_NO_CONTEXT;
    EGLSurface egl_surface_ = EGL_NO_SURFACE;
    mjvScene  sensor_scene_{};
    mjrContext sensor_context_{};
    mjvOption sensor_option_{};
    mjvCamera sensor_camera_{};
    std::vector<float> depth_buffers_;

    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

}  // namespace mujoco_gl_lidar

#endif  // MUJOCO_ROS2_CONTROL_MUJOCO_GL_LIDAR_HPP
