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

        sim->mnew_ = m;
        sim->dnew_ = d;
        mju::strcpy_arr(sim->filename, "Mujoco ros2 control");

        InitializeProfiler(sim.get());
        InitializeSensor(sim.get());

        if (!sim->is_passive_) {
            mjv_defaultScene(&sim->scn);
            mjv_makeScene(nullptr, &sim->scn, sim->kMaxGeom);
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
        sim->platform_ui->PollEvents();

        sim->Sync();
        sim->Render();
    }
}