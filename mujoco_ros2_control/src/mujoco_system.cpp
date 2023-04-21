// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "mujoco_ros2_control/mujoco_system.hpp"
//#include "gazebo/sensors/ImuSensor.hh"
//#include "gazebo/sensors/ForceTorqueSensor.hh"
//#include "gazebo/sensors/SensorManager.hh"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

struct MimicJoint {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
};

class mujoco_ros2_control::MujocoSystemPrivate {
public:
    MujocoSystemPrivate() = default;

    ~MujocoSystemPrivate() = default;

    /// \brief Degrees od freedom.
    size_t n_dof_;

    /// \brief Number of sensors.
    //size_t n_sensors_;

    /// \brief Mujoco Model Ptr.
    mjModel *mujoco_model_;
    mjData *mujoco_data_;
    const urdf::Model *urdf_model_;

    /// \brief last time the write method was called.
    rclcpp::Time last_update_sim_time_ros_;

    /// \brief vector with the joint's names.
    std::vector<std::string> joint_names_;

    /// \brief vector with the control method defined in the URDF for each joint.
    std::vector<MujocoSystemInterface::ControlMethod> joint_control_methods_;

    std::vector<control_toolbox::Pid> pid_controllers_;

    /// \brief handles to the joints from within Mujoco
    std::vector<MujocoSystem::MujocoJointData> sim_joints_;

    /// \brief vector with the current joint position
    std::vector<double> joint_position_;

    /// \brief vector with the current joint velocity
    std::vector<double> joint_velocity_;

    /// \brief vector with the current joint effort
    std::vector<double> joint_effort_;

    /// \brief vector with the current cmd joint position
    std::vector<double> joint_position_cmd_;

    /// \brief vector with the current cmd joint velocity
    std::vector<double> joint_velocity_cmd_;

    /// \brief vector with the current cmd joint effort
    std::vector<double> joint_effort_cmd_;

    /// \brief vector with the lower joint limits
    std::vector<double> joint_lower_limit_;

    /// \brief vector with the upper joint limits
    std::vector<double> joint_upper_limit_;

    /// \brief vector with the effort limits
    std::vector<double> joint_effort_limit_;

    /**
    /// \brief handles to the imus from within Gazebo
    std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

    /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
    std::vector<std::array<double, 10>> imu_sensor_data_;

    /// \brief handles to the FT sensors from within Gazebo
    std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

    /// \brief An array per FT sensor for 3D force and torquee
    std::vector<std::array<double, 6>> ft_sensor_data_;
    */
    /// \brief state interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::StateInterface> state_interfaces_;

    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    /// \brief mapping of mimicked joints to index of joint they mimic
    std::vector<MimicJoint> mimic_joints_;

    /// \brief Gain which converts position error to a velocity command
    //double position_proportional_gain_;
};


namespace mujoco_ros2_control {

    bool MujocoSystem::initSim(
            rclcpp::Node::SharedPtr &model_nh,
            mjModel *mujoco_model, mjData *mujoco_data,
            const hardware_interface::HardwareInfo &hardware_info,
            const urdf::Model *const urdf_model,
            uint objects_in_scene) {

        this->nh_ = model_nh;
        this->dataPtr = std::make_unique<MujocoSystemPrivate>();
        this->dataPtr->n_dof_ = mujoco_model_->njnt - objects_in_scene;
        this->dataPtr->urdf_model_ = urdf_model;
        this->dataPtr->mujoco_model_ = mujoco_model;
        this->dataPtr->mujoco_data_ = mujoco_data;
        RCLCPP_INFO(this->nh_->get_logger(), "%i robot degrees of freedom found.", n_dof_);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i generalized coordinates (qpos) found.", mujoco_model_->nq);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i degrees of freedom (qvel) found.", mujoco_model_->nv);
        RCLCPP_INFO(this->nh_->get_logger(), "%i actuators/controls (ctrl) found.", mujoco_model_->nu);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i actuation states (act) found.", mujoco_model_->na);
        RCLCPP_DEBUG(this->nh_->get_logger(), "%i joints (njnt) found.", mujoco_model_->njnt);

        registerJoints(hardware_info);
        //registerSensors(hardware_info, mujoco_model, mujoco_data);

        return true;
    }

    void MujocoSystem::registerJoints(const hardware_interface::HardwareInfo &hardware_info) {
        this->dataPtr->n_dof_ = hardware_info.joints.size();

        this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_effort_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_lower_limit_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_upper_limit_.resize(this->dataPtr->n_dof_);
        this->dataPtr->joint_effort_limit_.resize(this->dataPtr->n_dof_);
        for (size_t j = 0; j < this->dataPtr->n_dof_; j++) {
            auto &joint_info = hardware_info.joints[j];
            std::string joint_name = this->dataPtr->joint_names_[j] = joint_info.name;

            MujocoJointData mj_joint_data{};
            mj_joint_data.id = mj_name2id(this->dataPtr->mujoco_model_, mjOBJ_JOINT, joint_info.name.c_str());
            mj_joint_data.qpos_addr = mujoco_model_->jnt_qposadr[mj_joint_data.id];
            mj_joint_data.qvel_addr = mujoco_model_->jnt_dofadr[mj_joint_data.id];
            mj_joint_data.type = mujoco_model_->jnt_type[mj_joint_data.id];
            this->dataPtr->sim_joints_.push_back(mj_joint_data);


            auto get_initial_value = [this](const hardware_interface::InterfaceInfo &interface_info) {
                if (!interface_info.initial_value.empty()) {
                    double value = std::stod(interface_info.initial_value);
                    RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
                    return value;
                } else {
                    return 0.0;
                }
            };

            // register the state handles
            double initial_position = std::numeric_limits<double>::quiet_NaN();
            double initial_velocity = std::numeric_limits<double>::quiet_NaN();
            double initial_effort = std::numeric_limits<double>::quiet_NaN();
            //double lower_limit = std::numeric_limits<double>::quiet_NaN();
            //double upper_limit = std::numeric_limits<double>::quiet_NaN();
            //double effort_limit = std::numeric_limits<double>::quiet_NaN();

            for (size_t i = 0; i < joint_info.state_interfaces.size(); i++) {
                if (joint_info.state_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_POSITION,
                            &this->dataPtr->joint_position_[j]);
                    initial_position = std::stod(joint_info.state_interfaces[i].initial_value);
                    //lower_limit = std::stod(joint_info.state_interfaces[i].min);
                    //upper_limit = std::stod(joint_info.state_interfaces[i].max);
                    this->dataPtr->joint_position_[j] = initial_position;
                    //this->dataPtr->joint_lower_limit_[j] = lower_limit;
                    //this->dataPtr->joint_lower_limit_[j] = upper_limit;

                } else if (joint_info.state_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &this->dataPtr->joint_velocity_[j]);
                    initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
                    this->dataPtr->joint_velocity_[j] = initial_velocity;
                } else if (joint_info.state_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->state_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_EFFORT,
                            &this->dataPtr->joint_effort_[j]);
                    initial_effort = get_initial_value(joint_info.state_interfaces[i]);
                    //effort_limit = std::stod(joint_info.state_interfaces[i].max);
                    this->dataPtr->joint_effort_[j] = initial_effort;
                    //this->dataPtr->joint_effort_limit_[j] = effort_limit;
                }
            }
            // Register the command handles
            for (size_t i = 0; i < joint_info.command_interfaces.size(); i++) {
                if (joint_info.command_interfaces[i].name == "position") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_POSITION,
                            &this->dataPtr->joint_position_cmd_[j]);
                    if (!std::isnan(initial_position)) {
                        this->dataPtr->joint_position_cmd_[j] = initial_position;
                    }
                } else if (joint_info.command_interfaces[i].name == "velocity") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_VELOCITY,
                            &this->dataPtr->joint_velocity_cmd_[j]);
                    if (!std::isnan(initial_position)) {
                        this->dataPtr->joint_velocity_cmd_[j] = initial_velocity;
                    }
                } else if (joint_info.command_interfaces[i].name == "effort") {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->command_interfaces_.emplace_back(
                            joint_info.name,
                            hardware_interface::HW_IF_EFFORT,
                            &this->dataPtr->joint_effort_cmd_[j]);
                    if (!std::isnan(initial_effort)) {
                        this->dataPtr->joint_effort_cmd_[j] = initial_effort;
                    }
                }
                if (!std::isnan(initial_position)) {
                    this->dataPtr->joint_position_cmd_[j] = initial_position;
                    //this->dataPtr->joint_lower_limit_[j] = lower_limit;
                    //this->dataPtr->joint_lower_limit_[j] = upper_limit;
                }
                if (!std::isnan(initial_velocity)) {
                    this->dataPtr->joint_velocity_cmd_[j] = initial_velocity;
                }
                if (!std::isnan(initial_position)) {
                    this->dataPtr->joint_effort_cmd_[j] = initial_effort;
                    //this->dataPtr->joint_effort_limit_[j] = effort_limit;
                }
            }
        }

        for (int mujoco_actuator_id = 0; mujoco_actuator_id < this->dataPtr->mujoco_model_->nu; mujoco_actuator_id++) {
            std::string actuator_name = mj_id2name(this->dataPtr->mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);
            MujocoActuatorData mj_actuator_data{};
            mj_actuator_data.id = mujoco_actuator_id;
            mujoco_actuators_.insert(std::pair<std::string, MujocoActuatorData>(actuator_name, mj_actuator_data));
        }
        for (auto &mujoco_actuator: mujoco_actuators_) {
            RCLCPP_DEBUG(this->nh_->get_logger(), "%s: %s", mujoco_actuator.first.c_str(),
                         mujoco_actuator.second.to_string().c_str());
        }

        for (std::string name: this->dataPtr->joint_names_) {

        }
    }

    CallbackReturn
    MujocoSystem::on_init(const hardware_interface::HardwareInfo &system_info) {
        if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    MujocoSystem::export_state_interfaces() {
        return std::move(this->state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface>
    MujocoSystem::export_command_interfaces() {
        return std::move(this->command_interfaces_);
    }

    CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MujocoSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    MujocoSystem::perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) {

        for (size_t j = 0; j < this->dataPtr->joint_names_.size(); j++) {
            for (const std::string &interface_name: stop_interfaces) {
                // Clear joint control method bits corresponding to stop interfaces
                if (interface_name == (this->dataPtr->joint_names_[j] + "/" +
                                       hardware_interface::HW_IF_POSITION)) {
                    this->dataPtr->joint_control_methods_[j] &= static_cast<ControlMethod_>(VELOCITY & EFFORT);
                } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_VELOCITY)) {
                    this->dataPtr->joint_control_methods_[j] &= static_cast<ControlMethod_>(POSITION & EFFORT);
                } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_EFFORT)) {
                    this->dataPtr->joint_control_methods_[j] &=
                            static_cast<ControlMethod_>(POSITION & VELOCITY);
                }
            }

            // Set joint control method bits corresponding to start interfaces
            for (const std::string &interface_name: start_interfaces) {
                if (interface_name == (this->dataPtr->joint_names_[j] + "/" +
                                       hardware_interface::HW_IF_POSITION)) {
                    this->dataPtr->joint_control_methods_[j] |= POSITION;
                } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_VELOCITY)) {
                    this->dataPtr->joint_control_methods_[j] |= VELOCITY;
                } else if (interface_name == (this->dataPtr->joint_names_[j] + "/" + // NOLINT
                                              hardware_interface::HW_IF_EFFORT)) {
                    this->dataPtr->joint_control_methods_[j] |= EFFORT;
                }
            }
        }

        // mimic joint has the same control mode as mimicked joint
        for (const auto &mimic_joint: this->dataPtr->mimic_joints_) {
            this->dataPtr->joint_control_methods_[mimic_joint.joint_index] =
                    this->dataPtr->joint_control_methods_[mimic_joint.mimicked_joint_index];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (size_t j = 0; j < this->dataPtr->joint_names_.size(); j++) {
            if (string_ends_with(this->dataPtr->joint_names_[j], "FJ1") ||
                string_ends_with(this->dataPtr->joint_names_[j], "FJ2")) {
                std::string actuator_name = this->dataPtr->joint_names_[j];
                actuator_name[actuator_name.size() - 1] = '0';
                MujocoActuatorData &actuator = mujoco_actuators_.at(actuator_name);
                this->dataPtr->joint_effort_[j] = mujoco_data_->qfrc_actuator[actuator.id] / 2;
            } else {
                this->dataPtr->joint_effort_[j] = mujoco_data_->qfrc_applied[this->dataPtr->sim_joints_[j].qpos_addr];
            }
            if (this->dataPtr->sim_joints_[j].type == urdf::Joint::PRISMATIC) {
                this->dataPtr->joint_position_[j] = mujoco_data_->qpos[this->dataPtr->sim_joints_[j].qpos_addr];
            } else {
                this->dataPtr->joint_position_[j] += angles::shortest_angular_distance(
                        this->dataPtr->joint_position_[j], mujoco_data_->qpos[this->dataPtr->sim_joints_[j].qpos_addr]);
            }
            this->dataPtr->joint_velocity_[j] = mujoco_data_->qvel[this->dataPtr->sim_joints_[j].qvel_addr];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MujocoSystem::write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) {
        for (auto &actuator: mujoco_actuators_) {
            if (string_ends_with(actuator.first, "FJ0")) {
                std::string joint_1_name = actuator.first;
                std::string joint_2_name = actuator.first;
                joint_1_name[joint_1_name.size() - 1] = '1';
                joint_2_name[joint_2_name.size() - 1] = '2';
                size_t index_joint_1;
                size_t index_joint_2;

                auto it = find(this->dataPtr->joint_names_.begin(), this->dataPtr->joint_names_.end(), joint_1_name);
                if (it != this->dataPtr->joint_names_.end()) {
                    index_joint_1 = it - this->dataPtr->joint_names_.begin();
                } else {
                    continue;
                }
                it = find(this->dataPtr->joint_names_.begin(), this->dataPtr->joint_names_.end(), joint_2_name);
                if (it != this->dataPtr->joint_names_.end()) {
                    index_joint_2 = it - this->dataPtr->joint_names_.begin();
                } else {
                    continue;
                }
                if (this->dataPtr->joint_control_methods_[index_joint_1] & EFFORT) {
                    mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_effort_cmd_[index_joint_1] +
                                                             this->dataPtr->joint_effort_cmd_[index_joint_2];
                }

                if (this->dataPtr->joint_control_methods_[index_joint_1] & POSITION) {
                    mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_position_cmd_[index_joint_1] +
                                                             this->dataPtr->joint_position_cmd_[index_joint_2];
                }

                if (this->dataPtr->joint_control_methods_[index_joint_1] & VELOCITY) {
                    mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_velocity_cmd_[index_joint_1] +
                                                             this->dataPtr->joint_velocity_cmd_[index_joint_1];
                }
            }
            for (size_t j = 0; j < this->dataPtr->joint_names_.size(); j++) {
                if (actuator.first == this->dataPtr->joint_names_[j]) {
                    if (this->dataPtr->joint_control_methods_[j] & EFFORT) {
                        mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_effort_cmd_[j];
                    }
                    if (this->dataPtr->joint_control_methods_[j] & POSITION) {
                        mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_position_cmd_[j];
                    }
                    if (this->dataPtr->joint_control_methods_[j] & VELOCITY) {
                        mujoco_data_->ctrl[actuator.second.id] = this->dataPtr->joint_velocity_cmd_[j];
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    bool MujocoSystem::string_ends_with(const std::string &value, const std::string &ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT

PLUGINLIB_EXPORT_CLASS(
        mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)
