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
 */

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include "urdf/model.h"

#include "mujoco/mujoco.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

#include "hardware_interface/system_interface.hpp"

#include "rclcpp/rclcpp.hpp"

/**
 * @namespace mujoco_ros2_control
 * @brief Contains classes and interfaces for integrating Mujoco simulations with ROS 2 control.
 *
 * The `mujoco_ros2_control` namespace provides the necessary classes and interfaces for integrating Mujoco simulations
 * with the ROS 2 control framework. It includes the `MujocoSystemInterface` class, which serves as the API-level access
 * point for reading and commanding joint properties in the Mujoco simulation.
 *
 * This namespace is designed to be used alongside the ROS 2 control system to enable control of Mujoco-based robot
 * models. It provides the necessary abstractions and interfaces to bridge the gap between the Mujoco simulation and
 * the control framework, allowing for seamless integration and control of robot models in Mujoco.
 */
namespace mujoco_ros2_control
{
    /**
     * @class MujocoSystemInterface
     * @brief Provides API-level access to read and command joint properties in a Mujoco simulation.
     *
     * The MujocoSystemInterface class is a system interface that extends the hardware_interface::SystemInterface.
     * It provides an API-level access point to read and command joint properties in a Mujoco simulation.
     * This interface allows the Mujoco simulation to be integrated with the ROS 2 control framework.
     *
     * The MujocoSystemInterface class is designed to be implemented by classes that interact with the Mujoco simulation
     * and provide the necessary functionality to read and command joint properties. It defines a single pure virtual
     * method, initSim(), that initializes the Mujoco simulation and returns true if successful.
     */
    class MujocoSystemInterface
            : public hardware_interface::SystemInterface
    {
    public:
        /**
         * @brief Initializes the Mujoco simulation.
         *
         * This method initializes the Mujoco simulation by setting the Mujoco model and data pointers,
         * registering joints, and returning true if successful.
         *
         * @param mujoco_model A pointer to the Mujoco model.
         * @param mujoco_data A pointer to the Mujoco data.
         * @param hardware_info Hardware information.
         * @param urdf_model_ptr A pointer to the URDF model.
         * @return True if initialization is successful, false otherwise.
         */
        virtual bool initSim(
                mjModel* mujoco_model, mjData *mujoco_data,
                const hardware_interface::HardwareInfo & hardware_info,
                const urdf::Model *urdf_model_ptr) = 0;
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_INTERFACE_HPP_
