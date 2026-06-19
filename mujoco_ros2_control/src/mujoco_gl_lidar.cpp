/**
 * @file mujoco_gl_lidar.cpp
 * @brief Implementation of MujocoGLLidar (OpenGL depth-buffer lidar).
 *
 * @author Adrian Danzglock
 * @date 2026
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2026, DFKI GmbH
 */

#include "mujoco_gl_lidar/mujoco_gl_lidar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace mujoco_gl_lidar {

MujocoGLLidar::MujocoGLLidar(rclcpp::Node::SharedPtr &node,
                             mjModel_ *model,
                             mjData_ *data,
                             std::mutex *sim_mutex,
                             int site_id,
                             const std::string &name,
                             std::atomic<bool> *stop)
    : nh_(node),
      mujoco_model_(model),
      mujoco_data_(data),
      sim_mutex_(sim_mutex),
      stop_(stop),
      site_id_(site_id),
      name_(name),
      rng_(std::random_device{}()) {

    snapshot_data_ = mj_makeData(mujoco_model_);
    if (!snapshot_data_) {
        throw std::runtime_error("MujocoGLLidar: mj_makeData failed");
    }

    param_listener_ = std::make_shared<ParamListener>(nh_);
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();

    frequency_      = params_.frequency;
    range_min_      = params_.range_min;
    range_max_      = params_.range_max;
    h_min_          = params_.horizontal_min_angle;
    h_max_          = params_.horizontal_max_angle;
    h_samples_      = params_.horizontal_samples;
    v_min_          = params_.vertical_min_angle;
    v_max_          = params_.vertical_max_angle;
    v_samples_      = params_.vertical_samples;
    noise_stddev_   = params_.noise_stddev;
    depth_threshold_ = params_.depth_threshold;
    output_kind_    = (params_.output == "cloud") ? OutputKind::Cloud : OutputKind::Scan;

    if (noise_stddev_ > 0.0) {
        noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);
    }

    parent_body_id_ = mujoco_model_->site_bodyid[site_id_];
    const char *body_name_c = mj_id2name(mujoco_model_, mjOBJ_BODY, parent_body_id_);
    const std::string parent_body_name = body_name_c ? std::string(body_name_c) : "";
    frame_id_ = parent_body_name;

    // Cache the site's static pose within its parent body.
    site_pos_in_body_[0] = mujoco_model_->site_pos[3 * site_id_ + 0];
    site_pos_in_body_[1] = mujoco_model_->site_pos[3 * site_id_ + 1];
    site_pos_in_body_[2] = mujoco_model_->site_pos[3 * site_id_ + 2];
    mju_quat2Mat(site_mat_in_body_,
                 &mujoco_model_->site_quat[4 * site_id_]);

    // Decide how many slice renders we need. Each slice covers the full vertical FOV but only a fraction of the
    // horizontal FOV, so we can keep the render width (and thus depth buffer sparsity) manageable even for wide FOVs.
    const double pad = std::max(0.0, params_.fov_padding);
    const double total_h_fov = h_max_ - h_min_;
    constexpr double kMaxSliceFov = 1.745;  // ~100 deg
    n_slices_ = std::max(1, static_cast<int>(std::ceil(total_h_fov / kMaxSliceFov)));
    slice_width_ = total_h_fov / n_slices_;
    slice_render_h_fov_ = slice_width_ + 2.0 * pad;

    // Vertical render extents — asymmetric around the lidar's horizon when
    // [vertical_min_angle, vertical_max_angle] isn't centered on 0.
    if (output_kind_ == OutputKind::Scan) {
        // Single ring at pitch=0, with symmetric padding around it.
        v_min_padded_ = -pad - 0.005;
        v_max_padded_ =  pad + 0.005;
    } else {
        v_min_padded_ = v_min_ - pad;
        v_max_padded_ = v_max_ + pad;
    }
    v_fov_    = v_max_padded_ - v_min_padded_;
    v_center_ = 0.5 * (v_min_padded_ + v_max_padded_);
    cos_v_center_ = std::cos(v_center_);
    sin_v_center_ = std::sin(v_center_);

    if (slice_render_h_fov_ >= M_PI || v_fov_ >= M_PI) {
        throw std::runtime_error(
            "MujocoGLLidar '" + name_ + "': render FOV >= pi rad, "
            "which is not renderable. Reduce angle range or fov_padding.");
    }

    // Tilt the camera by v_center_ so the optical axis is centered on the
    // beam pitch range, which improves the effective vertical resolution.
    const double tan_half_v = std::tan(v_fov_ / 2.0);
    const double vert_ext   = 2.0 * tan_half_v;
    const double horiz_ext  = 2.0 * std::tan(slice_render_h_fov_ / 2.0);

    render_height_ = std::max(2, static_cast<int>(params_.render_height));
    const double aspect = horiz_ext / vert_ext;
    render_width_  = std::max(2, static_cast<int>(std::round(render_height_ * aspect)));

    RCLCPP_INFO(nh_->get_logger(),
                "MujocoGLLidar '%s': total_h_fov=%.1f deg in %d slices of %.1f deg, "
                "render %dx%d, aspect %.2f, v_fov %.1f deg "
                "(padded %.1f to %.1f, tilt %.1f)",
                name_.c_str(), total_h_fov * 180.0 / M_PI, n_slices_,
                slice_render_h_fov_ * 180.0 / M_PI,
                render_width_, render_height_, aspect,
                v_fov_ * 180.0 / M_PI,
                v_min_padded_ * 180.0 / M_PI, v_max_padded_ * 180.0 / M_PI,
                v_center_ * 180.0 / M_PI);

    // Symmetric intrinsics around the tilted axis: pitch=v_center maps to
    // the image center; pitch=v_min/v_max map to the bottom/top rows.
    f_  = render_height_ / vert_ext;
    cx_ = render_width_ / 2.0;
    cy_ = render_height_ / 2.0;

    // Clip distances. MuJoCo stores these as fractions of model.stat.extent.
    extent_     = mujoco_model_->stat.extent;
    znear_frac_ = mujoco_model_->vis.map.znear;
    zfar_frac_  = mujoco_model_->vis.map.zfar;
    znear_      = znear_frac_ * extent_;
    zfar_       = zfar_frac_  * extent_;

    depth_buffers_.assign(
        static_cast<size_t>(n_slices_) * render_width_ * render_height_, 1.0f);

    // Beam directions (lidar +X = forward, +Z = up) plus per-beam sampling
    auto build_beam = [&](double yaw, double pitch) {
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const std::array<double, 3> dl = {std::cos(yaw) * cp, std::sin(yaw) * cp, sp};
        beam_dirs_local_.push_back(dl);

        // Slice the beam belongs to (clamp guards numerical edges).
        int s = static_cast<int>(std::floor((yaw - h_min_) / slice_width_));
        if (s < 0) s = 0;
        if (s >= n_slices_) s = n_slices_ - 1;
        const double yaw_off = h_min_ + (s + 0.5) * slice_width_;
        const double yaw_in  = yaw - yaw_off;
        BeamSample bs;
        bs.slice = s;
        bs.dx = std::cos(yaw_in) * cp;
        bs.dy = std::sin(yaw_in) * cp;
        bs.dz = sp;
        beam_samples_.push_back(bs);
    };

    if (output_kind_ == OutputKind::Scan) {
        beam_dirs_local_.reserve(h_samples_);
        beam_samples_.reserve(h_samples_);
        const double dh = (h_samples_ > 1) ? (h_max_ - h_min_) / (h_samples_ - 1) : 0.0;
        for (int i = 0; i < h_samples_; ++i) {
            build_beam(h_min_ + dh * i, 0.0);
        }
        scan_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>(
            "/" + name_ + "/scan", 10);
    } else {
        beam_dirs_local_.reserve(static_cast<size_t>(v_samples_) * h_samples_);
        beam_samples_.reserve(static_cast<size_t>(v_samples_) * h_samples_);
        const double dh = (h_samples_ > 1) ? (h_max_ - h_min_) / (h_samples_ - 1) : 0.0;
        const double dv = (v_samples_ > 1) ? (v_max_ - v_min_) / (v_samples_ - 1) : 0.0;
        for (int r = 0; r < v_samples_; ++r) {
            for (int c = 0; c < h_samples_; ++c) {
                build_beam(h_min_ + dh * c, v_min_ + dv * r);
            }
        }
        cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/" + name_ + "/points", 10);
    }

    if (!glfwInit()) {
        throw std::runtime_error("MujocoGLLidar: glfwInit failed");
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(render_width_, render_height_, name_.c_str(), nullptr, nullptr);
    if (!window_) {
        throw std::runtime_error("MujocoGLLidar: glfwCreateWindow failed");
    }
    glfwSetWindowAttrib(window_, GLFW_RESIZABLE, GLFW_FALSE);

    auto prev_ctx = glfwGetCurrentContext();
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    sensor_camera_.type = mjCAMERA_USER;  // set scene.camera directly per frame
    mjr_defaultContext(&sensor_context_);
    mjv_defaultOption(&sensor_option_);
    mjv_defaultScene(&sensor_scene_);
    mjv_makeScene(mujoco_model_, &sensor_scene_, 2000);
    mjr_makeContext(mujoco_model_, &sensor_context_, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_WINDOW, &sensor_context_);

    glfwMakeContextCurrent(prev_ctx);

    RCLCPP_INFO(nh_->get_logger(),
                "MujocoGLLidar '%s' on site_id=%d ready: %s, %d beams%s @ %.1f Hz, "
                "%d slice(s) at %dx%d, total h_fov %.1f deg (%.1f deg/slice), "
                "v_fov %.1f deg",
                name_.c_str(), site_id_,
                output_kind_ == OutputKind::Scan ? "scan" : "cloud",
                h_samples_,
                output_kind_ == OutputKind::Cloud
                    ? (" x " + std::to_string(v_samples_) + " rings").c_str()
                    : "",
                frequency_,
                n_slices_, render_width_, render_height_,
                total_h_fov * 180.0 / M_PI,
                slice_render_h_fov_ * 180.0 / M_PI,
                v_fov_ * 180.0 / M_PI);
}

MujocoGLLidar::~MujocoGLLidar() {
    if (window_) {
        auto prev_ctx = glfwGetCurrentContext();
        glfwMakeContextCurrent(window_);
        mjv_freeScene(&sensor_scene_);
        mjr_freeContext(&sensor_context_);
        glfwMakeContextCurrent(prev_ctx);
        glfwDestroyWindow(window_);
    }
    if (snapshot_data_) {
        mj_deleteData(snapshot_data_);
        snapshot_data_ = nullptr;
    }
}

void MujocoGLLidar::render_depth() {
    const mjtNum *spos = snapshot_data_->site_xpos + 3 * site_id_;
    const mjtNum *smat = snapshot_data_->site_xmat + 9 * site_id_;

    const mjrRect viewport{0, 0, render_width_, render_height_};
    // Symmetric vertical frustum around the tilted optical axis.
    const double half_v_z = std::tan(v_fov_ / 2.0) * znear_;
    const size_t slice_pixels = static_cast<size_t>(render_width_) * render_height_;

    for (int s = 0; s < n_slices_; ++s) {
        const double yaw_off = h_min_ + (s + 0.5) * slice_width_;
        const double cy = std::cos(yaw_off);
        const double sy = std::sin(yaw_off);

        // Slice-local camera axes, tilted by v_center_ around slice +Y so the
        // optical axis sits at the middle of the configured pitch range
        const double fl_x =  cy * cos_v_center_;
        const double fl_y =  sy * cos_v_center_;
        const double fl_z =  sin_v_center_;
        const double ul_x = -cy * sin_v_center_;
        const double ul_y = -sy * sin_v_center_;
        const double ul_z =  cos_v_center_;

        // Transform to world
        mjtNum forward[3], up[3];
        forward[0] = smat[0] * fl_x + smat[1] * fl_y + smat[2] * fl_z;
        forward[1] = smat[3] * fl_x + smat[4] * fl_y + smat[5] * fl_z;
        forward[2] = smat[6] * fl_x + smat[7] * fl_y + smat[8] * fl_z;
        up[0]      = smat[0] * ul_x + smat[1] * ul_y + smat[2] * ul_z;
        up[1]      = smat[3] * ul_x + smat[4] * ul_y + smat[5] * ul_z;
        up[2]      = smat[6] * ul_x + smat[7] * ul_y + smat[8] * ul_z;

        for (int eye = 0; eye < 2; ++eye) {
            mjvGLCamera &gc = sensor_scene_.camera[eye];
            gc.pos[0]     = static_cast<float>(spos[0]);
            gc.pos[1]     = static_cast<float>(spos[1]);
            gc.pos[2]     = static_cast<float>(spos[2]);
            gc.forward[0] = static_cast<float>(forward[0]);
            gc.forward[1] = static_cast<float>(forward[1]);
            gc.forward[2] = static_cast<float>(forward[2]);
            gc.up[0]      = static_cast<float>(up[0]);
            gc.up[1]      = static_cast<float>(up[1]);
            gc.up[2]      = static_cast<float>(up[2]);
            gc.frustum_center = 0.0f;
            gc.frustum_bottom = static_cast<float>(-half_v_z);
            gc.frustum_top    = static_cast<float>( half_v_z);
            gc.frustum_near   = static_cast<float>(znear_);
            gc.frustum_far    = static_cast<float>(zfar_);
        }

        mjv_updateScene(mujoco_model_, snapshot_data_, &sensor_option_,
                        nullptr, &sensor_camera_, mjCAT_ALL, &sensor_scene_);
        mjr_render(viewport, &sensor_scene_, &sensor_context_);
        mjr_readPixels(nullptr,
                       depth_buffers_.data() + s * slice_pixels,
                       viewport, &sensor_context_);
    }
    glfwSwapBuffers(window_);
}

bool MujocoGLLidar::sample_beam(size_t beam_index, double &range,
                                double *cos_incidence) {
    const BeamSample &b = beam_samples_[beam_index];

    // Slice-local -> tilted-camera-local
    const double cx = -b.dy;
    const double cy = sin_v_center_ * b.dx - cos_v_center_ * b.dz;
    const double cz = cos_v_center_ * b.dx + sin_v_center_ * b.dz;

    if (cz <= 1e-6) return false;

    // Project
    const double u = cx_ + (cx / cz) * f_;
    const double v_top = cy_ + (cy / cz) * f_;

    if (u < 0.0 || u >= render_width_ - 1 ||
        v_top < 0.0 || v_top >= render_height_ - 1) {
        return false;
    }

    const double v_gl = (render_height_ - 1) - v_top;

    // Pixel coords
    const int u0 = static_cast<int>(std::floor(u));
    const int v0 = static_cast<int>(std::floor(v_gl));
    const int u1 = std::min(u0 + 1, render_width_ - 1);
    const int v1 = std::min(v0 + 1, render_height_ - 1);

    const double tu = u - u0;
    const double tv = v_gl - v0;

    const size_t slice_pixels =
        static_cast<size_t>(render_width_) * render_height_;
    const float *slice_buf =
        depth_buffers_.data() + b.slice * slice_pixels;

    auto sample = [&](int uu, int vv) {
        return static_cast<double>(slice_buf[vv * render_width_ + uu]);
    };

    const double d00 = sample(u0, v0);
    const double d10 = sample(u1, v0);
    const double d01 = sample(u0, v1);
    const double d11 = sample(u1, v1);

    // Edge-aware consistency check
    const double eps = 1e-6;
    double vals[4] = {d00, d10, d01, d11};

    double dmin = 1e9, dmax = -1e9;
    int valid_count = 0;

    for (double d : vals) {
        if (d < 1.0 - eps) {  // ignore far-plane
            dmin = std::min(dmin, d);
            dmax = std::max(dmax, d);
            valid_count++;
        }
    }

    if (valid_count == 0) return false;

    double raw;

    if (valid_count == 4 && (dmax - dmin) <= depth_threshold_) {
        // Use bilinear
        raw =
            (1 - tu) * (1 - tv) * d00 +
            tu       * (1 - tv) * d10 +
            (1 - tu) * tv       * d01 +
            tu       * tv       * d11;
    } else {
        // Fallback: nearest valid sample
        raw = d00;

        if (raw >= 1.0 - eps) {
            // try others if nearest is invalid
            if (d10 < 1.0 - eps) raw = d10;
            else if (d01 < 1.0 - eps) raw = d01;
            else if (d11 < 1.0 - eps) raw = d11;
            else return false;
        }
    }

    if (raw >= 1.0 - eps) return false;

    // Linearize depth
    const double linear_z =
        znear_frac_ * zfar_frac_ * extent_ /
        (zfar_frac_ - raw * (zfar_frac_ - znear_frac_));

    range = linear_z / cz;

    if (cos_incidence) {
        // Estimate surface normal from the depth gradient at the sample.
        // Only trustworthy when the four corners agree (same surface, no
        // depth edge); fall back to 1.0 otherwise.
        double cos_i = 1.0;
        if (valid_count == 4 && (dmax - dmin) <= depth_threshold_) {
            auto unproject = [&](int uu, int vv_gl, double raw_d) {
                const double z = znear_frac_ * zfar_frac_ * extent_ /
                                 (zfar_frac_ - raw_d * (zfar_frac_ - znear_frac_));
                const double v_top_pix = (render_height_ - 1) - vv_gl;
                return std::array<double, 3>{
                    (uu - cx_) / f_ * z,
                    (v_top_pix - cy_) / f_ * z,
                    z};
            };
            const auto P00 = unproject(u0, v0, d00);
            const auto P10 = unproject(u1, v0, d10);
            const auto P01 = unproject(u0, v1, d01);

            const std::array<double, 3> du{
                P10[0] - P00[0], P10[1] - P00[1], P10[2] - P00[2]};
            const std::array<double, 3> dv{
                P01[0] - P00[0], P01[1] - P00[1], P01[2] - P00[2]};
            const std::array<double, 3> n{
                du[1] * dv[2] - du[2] * dv[1],
                du[2] * dv[0] - du[0] * dv[2],
                du[0] * dv[1] - du[1] * dv[0]};

            const double nlen = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            const double blen = std::sqrt(cx*cx + cy*cy + cz*cz);
            if (nlen > 1e-12 && blen > 1e-12) {
                cos_i = std::abs(cx*n[0] + cy*n[1] + cz*n[2]) / (nlen * blen);
            }
        }
        *cos_incidence = cos_i;
    }
    return true;
}

void MujocoGLLidar::publish_scan(const rclcpp::Time &stamp) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.angle_min = static_cast<float>(h_min_);
    msg.angle_max = static_cast<float>(h_max_);
    msg.angle_increment = (h_samples_ > 1)
        ? static_cast<float>((h_max_ - h_min_) / (h_samples_ - 1))
        : 0.0f;
    msg.time_increment = 0.0f;
    msg.scan_time = static_cast<float>(1.0 / frequency_);
    msg.range_min = static_cast<float>(range_min_);
    msg.range_max = static_cast<float>(range_max_);
    msg.ranges.resize(h_samples_);
    msg.intensities.resize(h_samples_);

    const float invalid = std::numeric_limits<float>::infinity();
    for (int i = 0; i < h_samples_; ++i) {
        double range, cos_i;
        const bool hit = sample_beam(static_cast<size_t>(i), range, &cos_i);
        if (hit && noise_stddev_ > 0.0) range += noise_dist_(rng_);
        const bool in_range = hit && range >= range_min_ && range <= range_max_;
        msg.ranges[i]      = in_range ? static_cast<float>(range) : invalid;
        msg.intensities[i] = in_range ? static_cast<float>(cos_i) : 0.0f;
    }
    scan_pub_->publish(msg);
}

void MujocoGLLidar::publish_cloud(const rclcpp::Time &stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(v_samples_);
    msg.width  = static_cast<uint32_t>(h_samples_);
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2Fields(
        4,
        "x",         1, sensor_msgs::msg::PointField::FLOAT32,
        "y",         1, sensor_msgs::msg::PointField::FLOAT32,
        "z",         1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(static_cast<size_t>(v_samples_) * h_samples_);

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(msg, "intensity");

    const float nanf = std::numeric_limits<float>::quiet_NaN();
    for (size_t k = 0; k < beam_dirs_local_.size();
         ++k, ++ix, ++iy, ++iz, ++ii) {
        double range, cos_i;
        const bool hit = sample_beam(k, range, &cos_i);
        if (hit && noise_stddev_ > 0.0) range += noise_dist_(rng_);
        if (!hit || range < range_min_ || range > range_max_) {
            *ix = nanf; *iy = nanf; *iz = nanf; *ii = nanf;
            continue;
        }
        // Beam endpoint in site (lidar) frame.
        const auto &dl = beam_dirs_local_[k];
        const double ps0 = dl[0] * range;
        const double ps1 = dl[1] * range;
        const double ps2 = dl[2] * range;

        *ix = static_cast<float>(site_pos_in_body_[0] +
                                 site_mat_in_body_[0] * ps0 +
                                 site_mat_in_body_[1] * ps1 +
                                 site_mat_in_body_[2] * ps2);
        *iy = static_cast<float>(site_pos_in_body_[1] +
                                 site_mat_in_body_[3] * ps0 +
                                 site_mat_in_body_[4] * ps1 +
                                 site_mat_in_body_[5] * ps2);
        *iz = static_cast<float>(site_pos_in_body_[2] +
                                 site_mat_in_body_[6] * ps0 +
                                 site_mat_in_body_[7] * ps1 +
                                 site_mat_in_body_[8] * ps2);
        *ii = static_cast<float>(cos_i);
    }
    cloud_pub_->publish(msg);
}

void MujocoGLLidar::update() {
    using namespace std::chrono_literals;
    mjtNum last_update = mujoco_data_->time;
    const double period = 1.0 / frequency_;

    while (rclcpp::ok() && !stop_->load()) {
        if (mujoco_data_->time < last_update) last_update = mujoco_data_->time;
        if (mujoco_data_->time - last_update < period) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        last_update += period;
        if (mujoco_data_->time - last_update >= period) {
            last_update = mujoco_data_->time;
        }

        // Take a time-consistent snapshot of mjData under the sim mutex.
        {
            std::lock_guard<std::mutex> lock(*sim_mutex_);
            mj_copyData(snapshot_data_, mujoco_model_, mujoco_data_);
        }

        // Stamp from the snapshot's sim time so the published cloud's
        // timestamp matches the world state we actually rendered.
        const rclcpp::Time stamp(static_cast<int64_t>(snapshot_data_->time * 1e9), RCL_ROS_TIME);

        auto prev_ctx = glfwGetCurrentContext();
        glfwMakeContextCurrent(window_);
        render_depth();
        if (output_kind_ == OutputKind::Scan) {
            publish_scan(stamp);
        } else {
            publish_cloud(stamp);
        }
        glfwMakeContextCurrent(prev_ctx);
    }
}

}  // namespace mujoco_gl_lidar
