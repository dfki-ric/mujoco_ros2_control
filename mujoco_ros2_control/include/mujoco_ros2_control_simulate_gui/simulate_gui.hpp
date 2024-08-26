// Copyright 2021 DeepMind Technologies Limited
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
#include "simulate.h"
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

        std::unique_ptr<mj::Simulate> sim;
    };
}
#endif //MUJOCO_SIMULATE_GUI_HPP