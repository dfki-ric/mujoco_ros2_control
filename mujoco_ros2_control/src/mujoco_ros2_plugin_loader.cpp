/**
 * @file mujoco_ros2_plugin_loader.cpp
 * @brief Parses <mujoco_ros2_plugin> declarations and drives the plugins they name.
 *
 * @author Adrian Danzglock
 * @date 2026
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2026, DFKI GmbH
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

#include "mujoco_ros2_control/mujoco_ros2_plugin_loader.hpp"

#include <set>
#include <utility>

#include "tinyxml2.h"

namespace mujoco_ros2_control {

namespace {

/// The top-level URDF element declaring one of these plugins. Deliberately not
/// nested inside <mujoco>: xacro2mjcf.py copies that subtree into the MJCF.
constexpr const char *kPluginTag = "mujoco_ros2_plugin";

/// Whitespace around a <param> value is formatting, not part of the value.
std::string trim(const char *text) {
    if (!text) {
        return {};
    }
    const std::string value(text);
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool is_disabled(const MujocoRos2PluginInfo &info) {
    const auto enabled = info.parameters.find("enabled");
    if (enabled == info.parameters.end() || enabled->second.empty()) {
        return false;
    }
    return !(enabled->second == "true" || enabled->second == "True" ||
             enabled->second == "1");
}

}  // namespace

MujocoRos2PluginLoader::MujocoRos2PluginLoader()
    : loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoRos2PluginInterface") {}

MujocoRos2PluginLoader::~MujocoRos2PluginLoader() { shutdown(); }

std::vector<MujocoRos2PluginInfo> MujocoRos2PluginLoader::parse(
        const std::string &urdf, const rclcpp::Logger &logger) {
    std::vector<MujocoRos2PluginInfo> plugins;
    if (urdf.empty()) {
        return plugins;
    }

    tinyxml2::XMLDocument document;
    if (document.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
        RCLCPP_ERROR(logger,
            "Could not parse the robot description as XML, so no <%s> can be read: %s",
            kPluginTag, document.ErrorStr());
        return plugins;
    }

    const tinyxml2::XMLElement *robot = document.RootElement();
    if (!robot) {
        RCLCPP_ERROR(logger, "The robot description has no root element.");
        return plugins;
    }

    std::set<std::string> names;
    for (const tinyxml2::XMLElement *element = robot->FirstChildElement(kPluginTag);
         element != nullptr;
         element = element->NextSiblingElement(kPluginTag)) {

        const std::string name = trim(element->Attribute("name"));
        if (name.empty()) {
            RCLCPP_ERROR(logger,
                "A <%s> has no 'name' attribute, skipping it.", kPluginTag);
            continue;
        }
        // The name becomes a node name, so a duplicate would put two plugins in
        // one parameter namespace and publish both on the same topics.
        if (!names.insert(name).second) {
            RCLCPP_ERROR(logger,
                "<%s name=\"%s\"> is declared more than once, skipping the repeat.",
                kPluginTag, name.c_str());
            continue;
        }

        const std::string plugin = trim(element->Attribute("plugin"));
        if (plugin.empty()) {
            RCLCPP_ERROR(logger,
                "<%s name=\"%s\"> has no 'plugin' attribute, skipping it.",
                kPluginTag, name.c_str());
            continue;
        }

        MujocoRos2PluginInfo info;
        info.name = name;
        info.plugin = plugin;

        for (const tinyxml2::XMLElement *parameter = element->FirstChildElement("param");
             parameter != nullptr;
             parameter = parameter->NextSiblingElement("param")) {

            const std::string key = trim(parameter->Attribute("name"));
            if (key.empty()) {
                RCLCPP_WARN(logger,
                    "<%s name=\"%s\"> has a <param> without a 'name' attribute, ignoring it.",
                    kPluginTag, name.c_str());
                continue;
            }
            info.parameters[key] = trim(parameter->GetText());
        }

        plugins.push_back(std::move(info));
    }

    return plugins;
}

size_t MujocoRos2PluginLoader::registerPlugins(
        const std::string &urdf,
        mjModel *mujoco_model,
        mjData *mujoco_data,
        std::mutex *sim_mutex,
        std::mutex *step_mutex,
        const std::atomic<bool> *stop,
        const rclcpp::Logger &logger) {
    return registerPlugins(
        parse(urdf, logger), mujoco_model, mujoco_data, sim_mutex, step_mutex, stop, logger);
}

size_t MujocoRos2PluginLoader::registerPlugins(
        const std::vector<MujocoRos2PluginInfo> &declarations,
        mjModel *mujoco_model,
        mjData *mujoco_data,
        std::mutex *sim_mutex,
        std::mutex *step_mutex,
        const std::atomic<bool> *stop,
        const rclcpp::Logger &logger) {

    if (declarations.empty()) {
        return 0;
    }
    stop_ = stop;

    for (const auto &info : declarations) {
        if (is_disabled(info)) {
            RCLCPP_INFO(logger, "Plugin '%s' is disabled in the description, skipping it.",
                        info.name.c_str());
            continue;
        }

        // One node per plugin: its parameters and topics are namespaced by the
        // plugin name instead of sharing the simulation node's.
        rclcpp::Node::SharedPtr node;
        try {
            node = rclcpp::Node::make_shared(
                info.name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger,
                "Plugin '%s': the name cannot be used as a node name, skipping it: %s",
                info.name.c_str(), e.what());
            continue;
        }

        std::shared_ptr<MujocoRos2PluginInterface> plugin;
        try {
            plugin = loader_.createSharedInstance(info.plugin);
        } catch (const pluginlib::PluginlibException &e) {
            RCLCPP_ERROR(logger,
                "Plugin '%s': could not load plugin '%s', skipping it: %s",
                info.name.c_str(), info.plugin.c_str(), e.what());
            continue;
        }

        const MujocoRos2PluginInterface::Context context{
            node, mujoco_model, mujoco_data, sim_mutex, step_mutex, stop};

        if (!plugin->initialize(context, info)) {
            RCLCPP_ERROR(logger,
                "Plugin '%s': plugin '%s' rejected its configuration, skipping it.",
                info.name.c_str(), info.plugin.c_str());
            continue;
        }

        // Only now that a plugin exists is the executor worth creating.
        if (!executor_) {
            executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        }
        executor_->add_node(node);

        nodes_.push_back(node);
        plugins_.push_back(plugin);

        const bool stepped =
            plugin->execution() == MujocoRos2PluginInterface::Execution::Step;
        if (stepped) {
            // Driven by the simulation thread through beforeStep()/afterStep();
            // giving it a thread as well would race it against itself.
            stepped_.push_back(plugin.get());
        } else {
            threads_.emplace_back([instance = plugin] { instance->run(); });
        }

        // Deliberately no rate here: a plugin overriding run() paces itself and
        // ignores rate(), so reporting it would be wrong as often as it is right.
        // Plugins log their own rate.
        RCLCPP_INFO(logger, "Plugin '%s': loaded plugin '%s' (%s)",
                    info.name.c_str(), info.plugin.c_str(),
                    stepped ? "per physics step" : "own thread");
    }

    if (executor_) {
        // Spun at normal priority, deliberately not on the controller manager's
        // SCHED_FIFO executor: these callbacks are not realtime work.
        executor_thread_ = std::thread([this] { executor_->spin(); });
    }

    return plugins_.size();
}

void MujocoRos2PluginLoader::beforeStep(mjData *mujoco_data) {
    for (auto *plugin : stepped_) {
        plugin->beforeStep(mujoco_data);
    }
}

void MujocoRos2PluginLoader::afterStep(
        const mjData *mujoco_data, const rclcpp::Time &stamp) {
    for (auto *plugin : stepped_) {
        plugin->afterStep(mujoco_data, stamp);
    }
}

std::vector<MujocoRos2PluginLoader::FrameRequest>
MujocoRos2PluginLoader::requestSynchronousFrames() {
    std::vector<FrameRequest> requests;
    requests.reserve(plugins_.size());
    for (auto &plugin : plugins_) {
        requests.push_back({plugin.get(), plugin->requestSynchronousFrame()});
    }
    return requests;
}

bool MujocoRos2PluginLoader::waitForSynchronousFrames(
        const std::vector<FrameRequest> &requests,
        std::chrono::milliseconds timeout,
        std::string *timed_out) {
    for (const auto &request : requests) {
        if (!request.plugin->waitForSynchronousFrame(request.sequence, timeout)) {
            if (timed_out) {
                *timed_out = request.plugin->name();
            }
            return false;
        }
    }
    return true;
}

void MujocoRos2PluginLoader::shutdown() {
    // run() returns once the stop flag is set; joining before that would hang.
    if (stop_ && !stop_->load()) {
        RCLCPP_WARN(rclcpp::get_logger("MujocoRos2PluginLoader"),
            "Shutting down while the stop flag is still clear; plugin threads may not return.");
    }

    for (auto &thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads_.clear();

    if (executor_) {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        for (const auto &node : nodes_) {
            executor_->remove_node(node);
        }
        executor_.reset();
    }

    // Before the class loader unloads the libraries these came from. stepped_
    // holds raw pointers into plugins_, so it goes first.
    stepped_.clear();
    plugins_.clear();
    nodes_.clear();
}

}  // namespace mujoco_ros2_control
