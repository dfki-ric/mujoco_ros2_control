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

#include <memory>
#include <string>
#include <vector>
#include "urdf/urdf/model.h"

#include "mujoco/mujoco.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

#include "hardware_interface/system_interface.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control
{
    class MujocoResourceManager
            : public hardware_interface::ResourceManager
    {
    public:
        MujocoResourceManager(
            rclcpp::Node::SharedPtr & node, 
            mjModel *mujoco_model, mjData *mujoco_data)
        : hardware_interface::ResourceManager (
            node->get_node_clock_interface(), node->get_node_logging_interface()),
            robot_hw_sim_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
            logger_(node->get_logger().get_child("MujocoResourceManager")) {
            node_ = node;
            mujoco_model_ = mujoco_model;
            mujoco_data_ = mujoco_data;
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
                if(system->initSim(mujoco_model_, mujoco_data_, hw_info, &urdf_model)) {
                    // initialize hardware
                    import_component(std::move(system), hw_info);
                    // activate all components
                    rclcpp_lifecycle::State state(
                            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                            hardware_interface::lifecycle_state_names::ACTIVE);
                    set_component_state(hw_info.name, state);
                } else {
                    components_are_loaded_and_initialized_ = false;
                }
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
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_RESOURCE_MANAGER_HPP_