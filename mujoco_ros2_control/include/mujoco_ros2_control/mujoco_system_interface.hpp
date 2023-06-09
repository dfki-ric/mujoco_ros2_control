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


#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_

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

    // SystemInterface provides API-level access to read and command joint properties.
    class MujocoSystemInterface
            : public hardware_interface::SystemInterface
    {
    public:

        /**
         * Load and stores the required Datas and Pointers from MuJoCo and ROS2 to this class
         * @param mujoco_model      Pointer to the MuJoCo model
         * @param mujoco_data       Pointer to the MuJoCo data
         * @param hardware_info     Description of the ROS2 Control definitions
         * @param urdf_model_ptr    Pointer to parsed URDF model
         * @return return true if the setup was completed
         */
        virtual bool initSim(
                mjModel* mujoco_model, mjData *mujoco_data,
                const hardware_interface::HardwareInfo & hardware_info,
                const urdf::Model *urdf_model_ptr) = 0;

    protected:
        rclcpp::Node::SharedPtr nh_;
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_