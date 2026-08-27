/**
 * @file mujoco_system_interface.hpp
 * @brief Mujoco System Interface
 *
 * This file contains the declaration of the MujocoSystemInterface class, which provides API-level access to read and
 * command joint properties in a Mujoco simulation. It extends the hardware_interface::SystemInterface and is designed
 * to be implemented by classes that interact with the Mujoco simulation and integrate it with the ROS 2 control
 * framework.
 *
 * @author Adrian Danzglock
 * @date 2025
 *
* @license BSD 3-Clause License
* @copyright Copyright (c) 2025, DFKI GmbH
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
 */

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_RESOURCE_MANAGER_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_RESOURCE_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include "urdf/urdf/model.h"

#include "mujoco/mujoco.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

#include "hardware_interface/system_interface.hpp"
#if !defined(ROS_DISTRO_HUMBLE)
#include "hardware_interface/types/hardware_component_params.hpp"
#endif

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control
{
#if defined(ROS_DISTRO_HUMBLE)
    // Humble's hardware_interface::ResourceManager has no load_and_initialize_components()
    // hook to override: the URDF is already available synchronously (it comes from the
    // "robot_description" parameter, not a topic controller_manager waits on), so it is
    // loaded eagerly from load_and_initialize() below, before this manager is ever handed
    // to the ControllerManager constructor.
    class MujocoResourceManager
            : public hardware_interface::ResourceManager
    {
    public:
        MujocoResourceManager(
            rclcpp::Node::SharedPtr & node,
            mjModel *mujoco_model, mjData *mujoco_data,
            std::atomic<bool> *system_configured = nullptr)
        : hardware_interface::ResourceManager(),
            robot_hw_sim_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
            logger_(node->get_logger().get_child("MujocoResourceManager")) {
            node_ = node;
            mujoco_model_ = mujoco_model;
            mujoco_data_ = mujoco_data;
            system_configured_ = system_configured;
        }
        MujocoResourceManager(const MujocoResourceManager &) = delete;

        bool load_and_initialize(const std::string & urdf) {
            // On Jazzy+, an empty/never-published "robot_description" just means
            // load_and_initialize_components() is never called, and the controller
            // manager runs on happily with zero hardware components. Mirror that
            // here instead of handing urdfdom an empty document, which throws
            // uncaught and aborts the process.
            if (urdf.empty()) {
                if (system_configured_) {
                    system_configured_->store(true, std::memory_order_release);
                }
                return true;
            }

            bool components_are_loaded_and_initialized = true;

            urdf::Model urdf_model;
            try {
                urdf_model.initString(urdf);
            } catch (const std::runtime_error & ex) {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing URDF in mujoco_ros2_control plugin: %s",
                             ex.what());
                rclcpp::shutdown();
                return false;
            }

            try {
                load_urdf(urdf, false, false);
            } catch (...) {
                // Expected: this resource manager is not the one meant to load and
                // activate components below, only to hold them.
                RCLCPP_ERROR(node_->get_logger(), "Error initializing URDF to resource manager!");
            }

            const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

            for (const auto & hw_info : hardware_info) {
                const std::string hardware_type = hw_info.hardware_class_type;
                auto system = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(robot_hw_sim_loader_.createUnmanagedInstance(hardware_type));
                // Has to precede initSim(): sensor plugins are constructed there
                // and the component's own get_node() stays null until much later.
                system->setSimNode(node_);
                if(system->initSim(mujoco_model_, mujoco_data_, hw_info, &urdf_model)) {
                    import_component(std::move(system), hw_info);
                    // activate all components
                    rclcpp_lifecycle::State state(
                            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                            hardware_interface::lifecycle_state_names::ACTIVE);
                    set_component_state(hw_info.name, state);
                } else {
                    components_are_loaded_and_initialized = false;
                }
            }

            if (system_configured_) {
                system_configured_->store(components_are_loaded_and_initialized, std::memory_order_release);
            }

            return components_are_loaded_and_initialized;
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;

        /// \brief Interface loader
        pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_loader_;

        rclcpp::Logger logger_;
        mjModel* mujoco_model_;
        mjData* mujoco_data_;
        std::atomic<bool>* system_configured_ = nullptr;
    };
#else
    class MujocoResourceManager
            : public hardware_interface::ResourceManager
    {
    public:
        MujocoResourceManager(
            rclcpp::Node::SharedPtr & node,
            mjModel *mujoco_model, mjData *mujoco_data,
            std::atomic<bool> *system_configured = nullptr)
        : hardware_interface::ResourceManager (
            node->get_node_clock_interface(), node->get_node_logging_interface()),
            robot_hw_sim_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
            logger_(node->get_logger().get_child("MujocoResourceManager")) {
            node_ = node;
            mujoco_model_ = mujoco_model;
            mujoco_data_ = mujoco_data;
            system_configured_ = system_configured;
        }
        MujocoResourceManager(const MujocoResourceManager &) = delete;

        // Called from Controller Manager when robot description is initialized from callback
        bool load_and_initialize_components(
            const std::string & urdf,
            unsigned int update_rate) override {
            components_are_loaded_and_initialized_ = true;

            urdf::Model urdf_model;
            std::vector<hardware_interface::HardwareInfo> control_hardware;

            try {
                urdf_model.initString(urdf);
            } catch (const std::runtime_error & ex) {
                RCLCPP_ERROR(node_->get_logger(), "Error parsing URDF in mujoco_ros2_control plugin: %s",
                             ex.what());
                rclcpp::shutdown();
            }
            const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

            for (const auto & hw_info : hardware_info) {
                const std::string hardware_type = hw_info.hardware_plugin_name;
                auto system = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(robot_hw_sim_loader_.createUnmanagedInstance(hardware_type));
                // Has to precede initSim(): sensor plugins are constructed there
                // and the component's own get_node() stays null until much later.
                system->setSimNode(node_);
                if(system->initSim(mujoco_model_, mujoco_data_, hw_info, &urdf_model)) {
                    // initialize hardware
                    hardware_interface::HardwareComponentParams params;
                    params.hardware_info = hw_info;
                    params.clock = node_->get_node_clock_interface()->get_clock();
                    params.logger = node_->get_logger().get_child(hw_info.name);
                    import_component(std::move(system), params);
                    // activate all components
                    rclcpp_lifecycle::State state(
                            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                            hardware_interface::lifecycle_state_names::ACTIVE);
                    set_component_state(hw_info.name, state);
                } else {
                    components_are_loaded_and_initialized_ = false;
                }
            }

            if (system_configured_) {
                system_configured_->store(components_are_loaded_and_initialized_, std::memory_order_release);
            }

            return components_are_loaded_and_initialized_;
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;

        /// \brief Interface loader
        pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> robot_hw_sim_loader_;

        rclcpp::Logger logger_;
        mjModel* mujoco_model_;
        mjData* mujoco_data_;
        std::atomic<bool>* system_configured_ = nullptr;
    };
#endif

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_RESOURCE_MANAGER_HPP_