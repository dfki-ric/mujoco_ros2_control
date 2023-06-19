/**
 * @file mujoco_visualization.cpp
 * @brief This file contains the implementation of the MujocoVisualization class.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license GNU General Public License, version 3 (GPL-3.0)
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * This file is governed by the GNU General Public License, version 3 (GPL-3.0).
 * The GPL-3.0 is a copyleft license that allows users to use, modify, and distribute software
 * while ensuring that these freedoms are passed on to subsequent users. It requires that any
 *  derivative works or modifications of the software be licensed under the GPL-3.0 as well.
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see https://www.gnu.org/licenses/gpl-3.0.html.
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

#include "mujoco_ros2_control/mujoco_visualization.hpp"

namespace mujoco_visualization {

    void MujocoVisualization::init(mjModel_* model, mjData_* data) {
        m = model;
        d = data;
        // init GLFW
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
        }

        // create window, make OpenGL context current, request v-sync
        window = glfwCreateWindow(1200, 900, "MuJoCo ROS2", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        // create scene and context
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // install GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, &keyboard_cb);
        glfwSetCursorPosCallback(window, &mouse_move_cb);
        glfwSetMouseButtonCallback(window, &mouse_button_cb);
        glfwSetScrollCallback(window, &scroll_cb);


    }

    void MujocoVisualization::update() {

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // keyboard callback
    void MujocoVisualization::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
        // backspace: reset simulation
        if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
            mj_resetData(m, d);
            mj_forward(m, d);
        }
    }


    // mouse button callback
    void MujocoVisualization::mouse_button(GLFWwindow* window, int button, int act, int mods) {
        // update button state
        button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
        button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }


    // mouse move callback
    void MujocoVisualization::mouse_move(GLFWwindow* window, double xpos, double ypos) {

        // no buttons down: nothing to do
        if (!button_left && !button_middle && !button_right) {
            return;
        }

        // compute mouse displacement, save
        double dx = xpos - lastx;
        double dy = ypos - lasty;
        lastx = xpos;
        lasty = ypos;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if (button_right) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (button_left) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }

        // move camera
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
    }


    // scroll callback
    void MujocoVisualization::scroll(GLFWwindow* window, double xoffset, double yoffset) {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
    }

    void MujocoVisualization::scroll_cb(GLFWwindow *window, double xoffset, double yoffset) {
        getInstance().scroll(window, xoffset, yoffset);
    }

    void MujocoVisualization::mouse_move_cb(GLFWwindow *window, double xpos, double ypos) {
        getInstance().mouse_move(window, xpos, ypos);
    }

    void MujocoVisualization::mouse_button_cb(GLFWwindow *window, int button, int act, int mods) {
        getInstance().mouse_button(window, button, act, mods);

    }

    void MujocoVisualization::keyboard_cb(GLFWwindow *window, int key, int scancode, int act, int mods) {
        getInstance().keyboard(window, key, scancode, act, mods);
    }

    void MujocoVisualization::terminate() {
        //free visualization storage
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
    }
} // mujoco_visualization