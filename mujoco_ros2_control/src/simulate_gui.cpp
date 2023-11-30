#include "mujoco_visualization/simulate_gui.hpp"

namespace mujoco_simulate_gui {
    void MujocoSimulateGui::init(mjModel_ *model, mjData_ *data) {
        m = model;
        d = data;
        // scan for libraries in the plugin directory to load additional plugins
        scanPluginLibraries();

        mjv_defaultCamera(&cam);

        mjv_defaultOption(&opt);

        mjv_defaultPerturb(&pert);

        // simulate object encapsulates the UI
        sim = std::make_unique<mj::Simulate>(
                std::make_unique<mj::GlfwAdapter>(),
                &cam, &opt, &pert, /* is_passive = */ true
        );

        sim->Load(m, d, "Mujoco ROS2 Control");
        sim->RenderLoop();
    }

    void MujocoSimulateGui::terminate() {
        sim.reset();
    }

    std::string MujocoSimulateGui::getExecutableDir() {
        constexpr char kPathSep = '/';
        const char* path = "/proc/self/exe";
        std::string realpath = [&]() -> std::string {
            std::unique_ptr<char[]> realpath(nullptr);
            std::uint32_t buf_size = 128;
            bool success = false;
            while (!success) {
                realpath.reset(new(std::nothrow) char[buf_size]);
                if (!realpath) {
                    std::cerr << "cannot allocate memory to store executable path\n";
                    return "";
                }

                std::size_t written = readlink(path, realpath.get(), buf_size);
                if (written < buf_size) {
                    realpath.get()[written] = '\0';
                    success = true;
                } else if (written == -1) {
                    if (errno == EINVAL) {
                        // path is already not a symlink, just use it
                        return path;
                    }

                    std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
                    return "";
                } else {
                    // realpath is too small, grow and retry
                    buf_size *= 2;
                }
            }
            return realpath.get();
        }();

        if (realpath.empty()) {
            return "";
        }

        for (std::size_t i = realpath.size() - 1; i > 0; --i) {
            if (realpath.c_str()[i] == kPathSep) {
                return realpath.substr(0, i);
            }
        }

        // don't scan through the entire file system's root
        return "";
    }

    void MujocoSimulateGui::scanPluginLibraries() {
        // check and print plugins that are linked directly into the executable
        int nplugin = mjp_pluginCount();
        if (nplugin) {
            std::printf("Built-in plugins:\n");
            for (int i = 0; i < nplugin; ++i) {
                std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
            }
        }

        // define platform-specific strings
        const std::string sep = "/";

        // try to open the ${EXECDIR}/plugin directory
        // ${EXECDIR} is the directory containing the simulate binary itself
        const std::string executable_dir = getExecutableDir();
        if (executable_dir.empty()) {
            return;
        }

        const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
        mj_loadAllPluginLibraries(
                plugin_dir.c_str(), +[](const char *filename, int first, int count) {
                    std::printf("Plugins registered by library '%s':\n", filename);
                    for (int i = first; i < first + count; ++i) {
                        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
                    }
                });
    }

    void MujocoSimulateGui::update() {
        sim->Sync();
        sim->Render();
    }
}