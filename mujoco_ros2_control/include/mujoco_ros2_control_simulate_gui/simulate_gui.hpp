/**
 * @file simulate_gui.hpp
 * @brief This file contains the wrapper for the mujoco simulate gui.
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
 * This code is a modified version of the original code from DeepMind Technologies Limited.
 * https://github.com/google-deepmind/mujoco/blob/main/simulate/main.cc
 *
 * Original code licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MUJOCO_SIMULATE_GUI_HPP
#define MUJOCO_SIMULATE_GUI_HPP

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <simulate.h>
#include "glfw_adapter.h"

extern "C" {
#include <sys/errno.h>
#include <unistd.h>
}

namespace mujoco_simulate_gui {
    namespace mj = ::mujoco;

    class MujocoSimulateGui {
    public:
        /**
         * Initialize the MuJoCo visualization.
         * This method initializes the GLFW library, creates a window, and sets up the visualization data structures.
         * It also registers the GLFW callbacks for keyboard, mouse button, mouse move, and scroll events.
         * @param model Pointer to the MuJoCo model.
         * @param data Pointer to the MuJoCo data.
         */
        void init(mjModel_ *model, mjData_ *data);
        /**
         * Update the MuJoCo visualization.
         * This method updates the scene and renders it on the window.
         * It also swaps the OpenGL buffers, processes GUI events, and calls GLFW callbacks.
         */
        void update();
        /**
         * Terminate the MuJoCo visualization.
         * This method frees the memory allocated for the visualization data structures.
         */
        void terminate();
        /**
         * Get the instance of the MujocoVisualization class.
         * This method returns the singleton instance of the class.
         *
         * @return A reference to the MujocoVisualization instance.
         */
        static MujocoSimulateGui& getInstance()
        {
            static MujocoSimulateGui instance;
            return instance;
        }
        std::unique_ptr<mj::Simulate> sim;

    private:
        /**
         * Private constructor to enforce singleton pattern.
         */
        MujocoSimulateGui(void){};
        /**
         * Private copy constructor to enforce singleton pattern.
         */
        MujocoSimulateGui(MujocoSimulateGui const&);

    protected:

        /**
         * @brief Pointer to the MuJoCo model.
         * Holds the reference to the MuJoCo model used for visualization.
         */
        mjModel* m = NULL;

        /**
         * @brief Pointer to the MuJoCo data.
         * Holds the reference to the MuJoCo data used for visualization.
         */
        mjData* d = NULL;

        /**
         * @brief Abstract camera for visualization.
         * Represents the camera used for viewing the MuJoCo simulation.
         */
        mjvCamera cam;

        /**
         * @brief Visualization options.
         * Holds the options for configuring the visualization.
         */
        mjvOption opt;


        mjvPerturb pert;
    };
}
#endif //MUJOCO_SIMULATE_GUI_HPP