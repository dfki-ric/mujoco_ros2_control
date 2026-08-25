/**
 * @file mujoco_ros2_plugin_loader.hpp
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_LOADER_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_LOADER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"

namespace mujoco_ros2_control {

/**
 * @brief Owns the plugins declared as `<mujoco_ros2_plugin>` in the robot description.
 *
 * These sit outside every `<ros2_control>` block, so ros2_control's URDF parser
 * never sees them and no hardware component owns them. This class is their whole
 * lifecycle: it parses the declarations out of the raw description, creates one
 * node per declaration, loads the class it names, and runs it on its own thread.
 *
 * @par Threading
 * A plugin declaring MujocoRos2PluginInterface::Execution::Step gets no thread:
 * the simulation calls it through beforeStep()/afterStep() instead. Every other
 * plugin gets a thread running MujocoRos2PluginInterface::run(). The nodes
 * are spun by an executor owned here, on a thread of normal priority -- not the
 * controller manager's executor, which runs under SCHED_FIFO and must not be
 * given non-realtime callbacks to service. Neither the executor nor its thread
 * is created when no plugin is declared.
 *
 * @par Destruction order
 * `loader_` is declared before `plugins_` so the instances are destroyed before
 * the class loader unloads their libraries. shutdown() joins every thread before
 * any of that happens, and the destructor calls it; the simulation has to call
 * it explicitly before deleting mjModel/mjData, which the plugin threads read.
 */
class MujocoRos2PluginLoader {
public:
    MujocoRos2PluginLoader();
    ~MujocoRos2PluginLoader();

    MujocoRos2PluginLoader(const MujocoRos2PluginLoader &) = delete;
    MujocoRos2PluginLoader &operator=(const MujocoRos2PluginLoader &) = delete;

    /**
     * @brief Every `<mujoco_ros2_plugin>` in @p urdf, in declaration order.
     *
     * A declaration missing its `name` or `plugin` attribute, or repeating a
     * name already taken, is reported and dropped. Exposed separately from
     * registerPlugins() so it can be tested without a simulation.
     */
    static std::vector<MujocoRos2PluginInfo> parse(
            const std::string &urdf, const rclcpp::Logger &logger);

    /**
     * @brief Load, configure and start every declared plugin.
     *
     * A plugin that fails to load, or whose configure() rejects its
     * declaration, is reported and skipped; the simulation comes up with the
     * rest. A plugin declaring `<param name="enabled">false</param>` is skipped
     * silently, which is how a description can ship a plugin that is off by
     * default.
     *
     * @param urdf         The raw robot description, as passed to the node's
     *                     `robot_description` parameter. Empty is not an error:
     *                     no declarations can be found, so none are started.
     * @param mujoco_model The compiled model.
     * @param mujoco_data  Live simulation data, read by the plugins under @p sim_mutex.
     * @param sim_mutex    The mutex the simulation thread holds around mj_step().
     * @param step_mutex   The outer lock serialising reset, teleport and stepping.
     * @param stop         Shutdown flag; the plugin threads exit once it is set.
     * @param logger       Used for the loading report; each plugin logs through
     *                     its own node afterwards.
     * @return The number of plugins started.
     */
    size_t registerPlugins(
            const std::string &urdf,
            mjModel *mujoco_model,
            mjData *mujoco_data,
            std::mutex *sim_mutex,
            std::mutex *step_mutex,
            const std::atomic<bool> *stop,
            const rclcpp::Logger &logger);

    /**
     * @brief As above, for declarations that have already been parsed.
     *
     * The simulation parses them up front to learn which MuJoCo sites are
     * claimed, so that its own prefix-based camera and lidar discovery can skip
     * them. Passing the result back in avoids parsing the description twice and
     * reporting every malformed declaration twice with it.
     */
    size_t registerPlugins(
            const std::vector<MujocoRos2PluginInfo> &declarations,
            mjModel *mujoco_model,
            mjData *mujoco_data,
            std::mutex *sim_mutex,
            std::mutex *step_mutex,
            const std::atomic<bool> *stop,
            const rclcpp::Logger &logger);

    /**
     * @brief Run the Execution::Step plugins' beforeStep() hooks.
     *
     * Call from the simulation thread, immediately before mj_step(), inside the
     * same critical section: that exclusivity is what lets those plugins touch
     * mjData without locking. Returns immediately when none are stepped.
     */
    void beforeStep(mjData *mujoco_data);

    /** @brief As beforeStep(), immediately after mj_step(). */
    void afterStep(const mjData *mujoco_data, const rclcpp::Time &stamp);

    /// One pending output, as returned by requestSynchronousFrames().
    struct FrameRequest {
        MujocoRos2PluginInterface *plugin;
        std::uint64_t sequence;
    };

    /**
     * @brief Ask every plugin for an output covering the state just stepped to.
     *
     * Call after the final mj_step() of a StepSimulation batch and before
     * waiting, so the plugins produce their frames concurrently. Plugins with
     * nothing pending are included and cost a virtual call each.
     */
    std::vector<FrameRequest> requestSynchronousFrames();

    /**
     * @brief Wait for everything requestSynchronousFrames() asked for.
     *
     * @param requests What that call returned.
     * @param timeout  Per plugin, not for the batch as a whole.
     * @param timed_out Set to the offending plugin's name when this returns
     *                  false, so the caller can say which one stalled.
     * @return True when every request was satisfied.
     */
    bool waitForSynchronousFrames(
            const std::vector<FrameRequest> &requests,
            std::chrono::milliseconds timeout,
            std::string *timed_out = nullptr);

    /**
     * @brief Join every plugin thread and stop spinning their nodes.
     *
     * Has to be called, and to return, before mjModel or mjData are deleted:
     * the plugin threads read both. Relies on the `stop` flag given to
     * registerPlugins() already being set, since that is what makes run()
     * return. Idempotent.
     */
    void shutdown();

    /** @brief True when the description declared no plugin of this kind. */
    bool empty() const { return plugins_.empty(); }

private:
    /// Declared first: must outlive every instance created from it.
    pluginlib::ClassLoader<MujocoRos2PluginInterface> loader_;
    std::vector<std::shared_ptr<MujocoRos2PluginInterface>> plugins_;
    /// The Execution::Step subset of plugins_, in declaration order. Raw pointers
    /// because plugins_ owns them and outlives this list.
    std::vector<MujocoRos2PluginInterface *> stepped_;
    std::vector<rclcpp::Node::SharedPtr> nodes_;
    std::vector<std::thread> threads_;

    /// Spins the plugin nodes. Created with the first plugin, so a simulation
    /// without any declaration pays for no extra thread.
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    const std::atomic<bool> *stop_ = nullptr;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_LOADER_HPP_
