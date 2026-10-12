#include "mujoco_ros2_control_examples/ulab_low_level.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "mujoco_ros2_control_plugins/sensor_lookup.hpp"

namespace ulab_mujoco_plugin
{

using mujoco_ros2_control_plugins::declare_double_array_param;
using mujoco_ros2_control_plugins::declare_double_param;
using mujoco_ros2_control_plugins::declare_param;
using mujoco_ros2_control_plugins::declare_string_array_param;

namespace
{

double array_value(
  const std::vector<double> & values,
  std::size_t index,
  double fallback = 0.0)
{
  if (index >= values.size() || !std::isfinite(values[index])) {
    return fallback;
  }
  return values[index];
}

}  // namespace

std::string UlabLowLevel::string_param(
  const char * key, const std::string & fallback)
{
  return declare_param(node_, info_, key, fallback);
}

double UlabLowLevel::double_param(
  const char * key, double fallback)
{
  return declare_double_param(node_, info_, key, fallback);
}

std::vector<std::string> UlabLowLevel::string_array_param(
  const char * key)
{
  return declare_string_array_param(node_, info_, key);
}

std::vector<double> UlabLowLevel::double_array_param(
  const char * key)
{
  return declare_double_array_param(node_, info_, key);
}

bool UlabLowLevel::resolve_joint(
  const mjModel * model,
  const std::string & name,
  int * qpos_adr,
  int * dof_adr) const
{
  const int id = mj_name2id(model, mjOBJ_JOINT, name.c_str());

  if (id < 0) {
    RCLCPP_ERROR(logger_, "MuJoCo joint '%s' not found.", name.c_str());
    return false;
  }

  const int type = model->jnt_type[id];

  if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
    RCLCPP_ERROR(
      logger_,
      "Joint '%s' must be a hinge or slide joint.",
      name.c_str());
    return false;
  }

  *qpos_adr = model->jnt_qposadr[id];
  *dof_adr = model->jnt_dofadr[id];

  return true;
}

bool UlabLowLevel::bind_joints(const mjModel * model)
{
  const auto names = string_array_param("joints");

  if (names.empty()) {
    RCLCPP_ERROR(logger_, "Parameter 'joints' must not be empty.");
    return false;
  }

  const auto limits = double_array_param("effort_limits");

  if (limits.size() > 1 && limits.size() != names.size()) {
    RCLCPP_ERROR(
      logger_,
      "'effort_limits' must be empty, contain one value, "
      "or contain one value per joint.");
    return false;
  }

  joints_.clear();
  joints_.reserve(names.size());

  for (std::size_t i = 0; i < names.size(); ++i) {
    Joint joint;
    joint.name = names[i];

    if (!resolve_joint(
        model, joint.name, &joint.qpos_adr, &joint.dof_adr)) {
      return false;
    }

    if (limits.size() == 1) {
      joint.effort_limit = std::abs(limits[0]);
    } else if (!limits.empty()) {
      joint.effort_limit = std::abs(limits[i]);
    }

    for (const auto & previous : joints_) {
      if (previous.dof_adr == joint.dof_adr) {
        RCLCPP_ERROR(
          logger_, "Joint '%s' duplicates a configured DOF.",
          joint.name.c_str());
        return false;
      }
    }

    joints_.push_back(std::move(joint));
  }

  return true;
}

bool UlabLowLevel::configure(
  const Context & context,
  const mujoco_ros2_control::MujocoRos2PluginInfo & info)
{
  (void)info;

  node_ = context.node;
  logger_ = node_->get_logger();

  set_default_rate(500.0);
  set_default_rate(double_param("rate", rate()));

  if (!bind_joints(context.mujoco_model)) {
    return false;
  }

  base_body_name_ = string_param("base_body_name", "");

  if (!base_body_name_.empty()) {
    base_body_id_ = mj_name2id(
      context.mujoco_model,
      mjOBJ_BODY,
      base_body_name_.c_str());

    if (base_body_id_ < 0) {
      RCLCPP_ERROR(
        logger_, "MuJoCo body '%s' not found.",
        base_body_name_.c_str());
      return false;
    }
  }

  robot_control_msgs::msg::JointCommand initial_command;
  command_buffer_.writeFromNonRT(initial_command);

  const auto command_topic =
    string_param("command_topic", "/joint_command");

  const auto state_topic =
    string_param("state_topic", "/robot_state");

  command_sub_ =
    node_->create_subscription<
      robot_control_msgs::msg::JointCommand>(
      command_topic,
      rclcpp::SystemDefaultsQoS(),
      [this](
        const robot_control_msgs::msg::JointCommand::SharedPtr msg)
      {
        command_buffer_.writeFromNonRT(*msg);
        holding_.store(false, std::memory_order_release);
      });

  auto publisher =
    node_->create_publisher<robot_control_msgs::msg::RobotState>(
      state_topic, rclcpp::SystemDefaultsQoS());

  state_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<
      robot_control_msgs::msg::RobotState>>(publisher);

  // Allocate the variable-length arrays before entering the physics loop.
  auto & state = state_pub_->msg_;

  state.joint_state.position.resize(joints_.size(), 0.0);
  state.joint_state.velocity.resize(joints_.size(), 0.0);
  state.joint_state.acceleration.resize(joints_.size(), 0.0);
  state.joint_state.effort.resize(joints_.size(), 0.0);

  RCLCPP_INFO(
    logger_,
    "Configured %zu joints; command topic '%s', state topic '%s'.",
    joints_.size(),
    command_topic.c_str(),
    state_topic.c_str());

  return true;
}

double UlabLowLevel::compute_torque(
  const Joint & joint,
  const mjData * data,
  const robot_control_msgs::msg::JointCommand & command,
  std::size_t index) const
{
  const double q = data->qpos[joint.qpos_adr];
  const double dq = data->qvel[joint.dof_adr];

  const double q_des = array_value(command.position, index);
  const double dq_des = array_value(command.velocity, index);
  const double kp = array_value(command.kp, index);
  const double kd = array_value(command.kd, index);
  const double ff = array_value(command.effort, index);

  double tau = kp * (q_des - q) + kd * (dq_des - dq) + ff;

  if (!std::isfinite(tau)) {
    tau = 0.0;
  }

  if (joint.effort_limit > 0.0) {
    tau = std::clamp(
      tau, -joint.effort_limit, joint.effort_limit);
  }

  return tau;
}

void UlabLowLevel::beforeStep(mjData * data)
{
  if (data->time < last_sim_time_) {
    holding_.store(true, std::memory_order_release);
    tick_ = 0;
  }

  last_sim_time_ = data->time;

  // MuJoCo does not automatically clear qfrc_applied each step.
  // Explicitly write zero when holding or when no command has arrived.
  if (holding_.load(std::memory_order_acquire)) {
    for (const auto & joint : joints_) {
      data->qfrc_applied[joint.dof_adr] = 0.0;
    }
    return;
  }

  const auto & command = *command_buffer_.readFromRT();

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    data->qfrc_applied[joints_[i].dof_adr] =
      compute_torque(joints_[i], data, command, i);
  }
}

void UlabLowLevel::afterStep(
  const mjData * data,
  const rclcpp::Time & stamp)
{
  if (!due(data->time)) {
    return;
  }

  if (!state_pub_ || !state_pub_->trylock()) {
    return;
  }

  auto & state = state_pub_->msg_;

  state.stamp = stamp;
  state.joint_state.stamp = state.stamp;

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = joints_[i];
    const int qpos = joint.qpos_adr;
    const int dof = joint.dof_adr;

    state.joint_state.position[i] = data->qpos[qpos];
    state.joint_state.velocity[i] = data->qvel[dof];
    state.joint_state.acceleration[i] = data->qacc[dof];

    // Generalized force from actuators plus the torque applied by this plugin.
    state.joint_state.effort[i] =
      data->qfrc_actuator[dof] + data->qfrc_applied[dof];
  }

  if (base_body_id_ >= 0) {
    const int body = base_body_id_;

    auto & pose = state.floating_base_state.pose;
    auto & twist = state.floating_base_state.twist;
    auto & acceleration = state.floating_base_state.acceleration;

    pose.position.x = data->xpos[3 * body + 0];
    pose.position.y = data->xpos[3 * body + 1];
    pose.position.z = data->xpos[3 * body + 2];

    // MuJoCo quaternion ordering is w, x, y, z.
    pose.orientation.w = data->xquat[4 * body + 0];
    pose.orientation.x = data->xquat[4 * body + 1];
    pose.orientation.y = data->xquat[4 * body + 2];
    pose.orientation.z = data->xquat[4 * body + 3];

    const mjtNum * cvel = data->cvel + 6 * body;
    const mjtNum * cacc = data->cacc + 6 * body;

    twist.angular.x = cvel[0];
    twist.angular.y = cvel[1];
    twist.angular.z = cvel[2];

    twist.linear.x = cvel[3];
    twist.linear.y = cvel[4];
    twist.linear.z = cvel[5];

    acceleration.angular.x = cacc[0];
    acceleration.angular.y = cacc[1];
    acceleration.angular.z = cacc[2];

    acceleration.linear.x = cacc[3];
    acceleration.linear.y = cacc[4];
    acceleration.linear.z = cacc[5];
  }

  // Contacts are left default-initialized because no Contacts schema
  // or contact mapping has been supplied yet.

  ++tick_;
  state_pub_->unlockAndPublish();
}

}  // namespace ulab_mujoco_plugin

PLUGINLIB_EXPORT_CLASS(
  ulab_mujoco_plugin::UlabLowLevel,
  mujoco_ros2_control::MujocoRos2PluginInterface)
