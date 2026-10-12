#ifndef ULAB_MUJOCO_PLUGIN__ULAB_LOW_LEVEL_HPP_
#define ULAB_MUJOCO_PLUGIN__ULAB_LOW_LEVEL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "robot_control_msgs/msg/joint_command.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"

namespace ulab_mujoco_plugin
{

class UlabLowLevel : public mujoco_ros2_control::MujocoRos2PluginInterface
{
public:
  Execution execution() const override
  {
    return Execution::Step;
  }

  bool configure(
    const Context & context,
    const mujoco_ros2_control::MujocoRos2PluginInfo & info) override;

  void beforeStep(mjData * mujoco_data) override;

  void afterStep(
    const mjData * mujoco_data,
    const rclcpp::Time & stamp) override;

private:
  struct Joint
  {
    std::string name;
    int qpos_adr{-1};
    int dof_adr{-1};
    double effort_limit{0.0};
  };

  bool bind_joints(const mjModel * model);
  bool resolve_joint(
    const mjModel * model,
    const std::string & name,
    int * qpos_adr,
    int * dof_adr) const;

  std::string string_param(
    const char * key, const std::string & fallback);
  double double_param(const char * key, double fallback);
  std::vector<std::string> string_array_param(const char * key);
  std::vector<double> double_array_param(const char * key);

  double compute_torque(
    const Joint & joint,
    const mjData * data,
    const robot_control_msgs::msg::JointCommand & command,
    std::size_t index) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ulab_low_level")};

  std::vector<Joint> joints_;
  std::string base_body_name_;
  int base_body_id_{-1};

  rclcpp::Subscription<
    robot_control_msgs::msg::JointCommand>::SharedPtr command_sub_;

  std::shared_ptr<
    realtime_tools::RealtimePublisher<
      robot_control_msgs::msg::RobotState>> state_pub_;

  realtime_tools::RealtimeBuffer<
    robot_control_msgs::msg::JointCommand> command_buffer_;

  std::atomic<bool> holding_{true};

  mjtNum last_sim_time_{-1.0};
  std::uint32_t tick_{0};
};

}  // namespace ulab_mujoco_plugin

#endif
