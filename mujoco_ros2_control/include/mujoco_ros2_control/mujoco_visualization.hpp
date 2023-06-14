/**
 * @file mujoco_visualization.hpp
 * @brief This file contains the implementation of the MujocoVisualization class.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license Apache License, Version 2.0
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
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
 *
 * This code is a modified version of the original code from DeepMind Technologies Limited.
 * https://github.com/deepmind/mujoco/blob/main/sample/basic.cc
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

#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
namespace mujoco_visualization {
    /**
     * @brief The MujocoVisualization class provides a wrapper for visualizing MuJoCo simulations using GLFW and OpenGL.
     *
     * The class encapsulates the necessary data structures and functions to initialize, update, and terminate the visualization.
     * It provides methods for handling keyboard and mouse input, rendering the scene, and interacting with the visualization window.
     * The visualization is based on the MuJoCo model and data structures, allowing real-time rendering of the simulation state.
     *
     * To use this class, you need to call the init() method to initialize the visualization, followed by the update() method
     * in a loop to continuously update and render the visualization. Finally, call the terminate() method to clean up resources
     * and free memory when the visualization is no longer needed.
     *
     * The MujocoVisualization class follows the singleton design pattern, ensuring that only one instance of the visualization
     * is created and accessed through the getInstance() method. This allows convenient access to the visualization from anywhere
     * in the code without needing to pass around object references.
     *
     * Note: This class relies on GLFW and OpenGL, and assumes that the necessary dependencies are properly installed.
     */
    class MujocoVisualization {
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
        static MujocoVisualization& getInstance()
        {
            static MujocoVisualization instance;
            return instance;
        }

    private:
        /**
         * Private constructor to enforce singleton pattern.
         */
        MujocoVisualization(void){};
        /**
         * Private copy constructor to enforce singleton pattern.
         */
        MujocoVisualization(MujocoVisualization const&);
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

        /**
         * @brief Abstract scene for visualization.
         * Represents the scene that contains the objects to be visualized.
         */
        mjvScene scn;

        /**
         * @brief Custom GPU context for visualization.
         * Holds the context for rendering the visualization using GPU.
         */
        mjrContext con;

        /**
         * @brief Pointer to the GLFW window.
         * Holds the pointer to the GLFW window used for visualization.
         */
        GLFWwindow* window = NULL;

        /**
         * @brief State of the left mouse button.
         * Indicates whether the left mouse button is currently pressed or not.
         */
        bool button_left = false;

        /**
         * @brief State of the middle mouse button.
         * Indicates whether the middle mouse button is currently pressed or not.
         */
        bool button_middle = false;

        /**
         * @brief State of the right mouse button.
         * Indicates whether the right mouse button is currently pressed or not.
         */
        bool button_right = false;

        /**
         * @brief Last known x-coordinate of the mouse position.
         * Holds the x-coordinate of the mouse position in the previous update.
         */
        double lastx = 0;

        /**
         * @brief Last known y-coordinate of the mouse position.
         * Holds the y-coordinate of the mouse position in the previous update.
         */
        double lasty = 0;

        /**
         * @brief Callback function for handling scroll events.
         * @param window The GLFW window
         * @param xoffset The scroll offset along the x-axis
         * @param yoffset The scroll offset along the y-axis
         */
        void scroll(GLFWwindow *window, double xoffset, double yoffset);
        /**
         * @brief Callback function for handling mouse move events.
         * @param window The GLFW window
         * @param xpos The new cursor x-coordinate
         * @param ypos The new cursor y-coordinate
         */
        void mouse_move(GLFWwindow *window, double xpos, double ypos);

        /**
         * @brief Callback function for handling mouse button events.
         * @param window The GLFW window
         * @param button The mouse button that was pressed or released
         * @param act The button action (GLFW_PRESS, GLFW_RELEASE)
         * @param mods Bit field describing which modifier keys were held down
         */
        void mouse_button(GLFWwindow *window, int button, int act, int mods);

        /**
         * @brief Callback function for handling keyboard events.
         * @param window The GLFW window
         * @param key The key that was pressed or released
         * @param scancode The system-specific scancode
         * @param act The key action (GLFW_PRESS, GLFW_RELEASE, GLFW_REPEAT)
         * @param mods Bit field describing which modifier keys were held down
         */
        void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);

        /**
         * @brief Callback function for handling scroll events.
         * @param window The GLFW window
         * @param xoffset The scroll offset along the x-axis
         * @param yoffset The scroll offset along the y-axis
         */
        static void scroll_cb(GLFWwindow *window, double xoffset, double yoffset);

        /**
         * @brief Callback function for handling mouse move events.
         * @param window The GLFW window
         * @param xpos The new cursor x-coordinate
         * @param ypos The new cursor y-coordinate
         */
        static void mouse_move_cb(GLFWwindow *window, double xpos, double ypos);

        /**
         * @brief Callback function for handling mouse button events.
         * @param window The GLFW window
         * @param button The mouse button that was pressed or released
         * @param act The button action (GLFW_PRESS, GLFW_RELEASE)
         * @param mods Bit field describing which modifier keys were held down
         */
        static void mouse_button_cb(GLFWwindow *window, int button, int act, int mods);

        /**
         * @brief Callback function for handling keyboard events.
         * @param window The GLFW window
         * @param key The key that was pressed or released
         * @param scancode The system-specific scancode
         * @param act The key action (GLFW_PRESS, GLFW_RELEASE, GLFW_REPEAT)
         * @param mods Bit field describing which modifier keys were held down
         */
        static void keyboard_cb(GLFWwindow *window, int key, int scancode, int act, int mods);

    };

} // mujoco_visualization

#endif //MUJOCO_ROS2_CONTROL_MUJOCO_VISUALIZATION_HPP
