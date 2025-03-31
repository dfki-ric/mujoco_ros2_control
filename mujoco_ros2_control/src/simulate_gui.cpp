/**
 * @file simulate_gui.cpp
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

#include "mujoco_ros2_control_simulate_gui/simulate_gui.hpp"
#include "simulate.cc"


namespace mujoco_simulate_gui {
    void MujocoSimulateGui::init(mjModel_ *model, mjData_ *data) {
        m = model;
        d = data;

        mjv_defaultCamera(&cam);

        mjv_defaultOption(&opt);

        mjv_defaultPerturb(&pert);

        // simulate object encapsulates the UI
        sim = std::make_unique<mj::Simulate>(
                std::make_unique<mj::GlfwAdapter>(),
                &cam, &opt, &pert, /* is_passive = */ false
        );

        sim->platform_ui->SetWindowTitle("MuJoCo ROS2");

        sim->mnew_ = m;
        sim->dnew_ = d;
        mju::strcpy_arr(sim->filename, "MuJoCo ROS2");

        InitializeProfiler(sim.get());
        InitializeSensor(sim.get());

        if (!sim->is_passive_) {
            mjv_defaultScene(&sim->scn);
            mjv_makeScene(m, &sim->scn, sim->kMaxGeom);
        }

        if (!sim->platform_ui->IsGPUAccelerated()) {
            sim->scn.flags[mjRND_SHADOW] = 0;
            sim->scn.flags[mjRND_REFLECTION] = 0;
        }

        // select default font
        int fontscale = ComputeFontScale(*sim->platform_ui);
        sim->font = fontscale/50 - 1;

        // make empty context
        sim->platform_ui->RefreshMjrContext(sim->m_, fontscale);

        // init state and uis
        std::memset(&sim->uistate, 0, sizeof(mjuiState));
        std::memset(&sim->ui0, 0, sizeof(mjUI));
        std::memset(&sim->ui1, 0, sizeof(mjUI));

        auto [buf_width, buf_height] = sim->platform_ui->GetFramebufferSize();
        sim->uistate.nrect = 1;
        sim->uistate.rect[0].width = buf_width;
        sim->uistate.rect[0].height = buf_height;

        sim->ui0.spacing = mjui_themeSpacing(sim->spacing);
        sim->ui0.color = mjui_themeColor(sim->color);
        sim->ui0.predicate = UiPredicate;
        sim->ui0.rectid = 1;
        sim->ui0.auxid = 0;

        sim->ui1.spacing = mjui_themeSpacing(sim->spacing);
        sim->ui1.color = mjui_themeColor(sim->color);
        sim->ui1.predicate = UiPredicate;
        sim->ui1.rectid = 2;
        sim->ui1.auxid = 1;

        // set GUI adapter callbacks
        sim->uistate.userdata = sim.get();
        sim->platform_ui->SetEventCallback(UiEvent);
        sim->platform_ui->SetLayoutCallback(UiLayout);

        // populate uis with standard sections
        sim->ui0.userdata = sim.get();
        sim->ui1.userdata = sim.get();
        mjui_add(&sim->ui0, defFile);
        mjui_add(&sim->ui0, sim->def_option);
        mjui_add(&sim->ui0, sim->def_simulation);
        mjui_add(&sim->ui0, sim->def_watch);
        UiModify(&sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
        UiModify(&sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());

        // set VSync to initial value
        sim->platform_ui->SetVSync(sim->vsync);

        sim->LoadOnRenderThread();
    }

    void MujocoSimulateGui::terminate() {
        sim.reset();
    }

    void MujocoSimulateGui::update() {
        //sim->platform_ui->UpdateMjuiState();
        {
            const mujoco::MutexLock lock(sim->mtx);
            sim->platform_ui->PollEvents();

            sim->Sync();
        }
        sim->Render();
    }
}