/**
 * @file mujoco_ros2_plugin_interface.hpp
 * @brief Base class for pluginlib-loaded plugins declared outside <ros2_control>.
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco/mujoco.h"

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control {

/**
 * @brief One `<mujoco_ros2_plugin>` element, as parsed from the robot description.
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="pad_touch" plugin="mujoco_ros2_control_examples/TouchGridSensor">
 *   <param name="site">pad_touch_site</param>
 *   <param name="rate">100.0</param>
 * </mujoco_ros2_plugin>
 * @endcode
 *
 * Unlike a `<sensor>` inside `<ros2_control>`, the class name is an attribute:
 * this element is parsed by MujocoRos2PluginLoader rather than by
 * ros2_control's URDF parser, which drops attributes it does not know.
 */
struct MujocoRos2PluginInfo {
    /// `name` attribute. Names the plugin's node, and with it the default
    /// namespace of everything the plugin publishes. Has to be unique.
    std::string name;
    /// `plugin` attribute: the pluginlib class to load.
    std::string plugin;
    /// `<param name="key">value</param>` children, keyed by name.
    std::map<std::string, std::string> parameters;
};

/**
 * @brief Base class for plugins that are not ros2_control components.
 *
 * The counterpart to MujocoRos2ControlSensorInterface. That one binds a
 * `<sensor>` inside a `<ros2_control>` block, exports scalar StateInterfaces,
 * and is read by the hardware component in the control loop. This one is for
 * anything that has no business being in that block -- a sensor whose output
 * does not fit that model, or something that is not a sensor at all:
 *
 * - It exports nothing to ros2_control, so nothing about it belongs in a
 *   description of the hardware's interfaces, and it does not need a
 *   `<ros2_control>` block to exist at all.
 * - It runs on its own thread at its own rate, decoupled from the controller
 *   manager's update rate.
 * - It gets its own node, so its parameters and topics are namespaced by the
 *   declaration's name instead of sharing the simulation node's.
 *
 * Cameras, lidars and tactile arrays are the motivating cases: a `touch_grid`
 * publishing `nchannel * width * height` values per sample has no scalar
 * interfaces to export, and a depth camera should not be paced by the control
 * loop.
 *
 * @par Execution model
 * A plugin picks one of two, by overriding execution():
 *
 * - Execution::Thread (the default) runs it on a thread of its own, so slow work
 *   never holds up the physics. Anything that renders belongs here.
 * - Execution::Step runs it on the simulation thread, inside the same critical
 *   section as mj_step(). No locking, exactly one call per physics step, and it
 *   stays in lockstep with service-driven stepping in synchronous mode -- at the
 *   price of delaying the step by however long it takes.
 *
 * The mode is the plugin's decision and cannot be overridden from the URDF: a
 * renderer forced into Execution::Step would destroy the simulation rate, and
 * the author always knows which model the plugin needs.
 *
 * @par What to implement
 * configure() always. Then, for Execution::Thread, either update() -- which the
 * default run() calls at rate() with the simulation mutex held -- or run()
 * itself when the plugin needs to own its scheduling. For Execution::Step,
 * beforeStep() and/or afterStep(), decimated with due() if it should not act on
 * every step.
 *
 * @par Lifetime
 * Instances are owned by MujocoRos2PluginLoader, which outlives them, and are
 * destroyed after their thread has been joined. Every member of Context outlives
 * the instance.
 */
class MujocoRos2PluginInterface {
public:
    /**
     * @brief Where a plugin runs. See @ref MujocoRos2PluginInterface "Execution model".
     */
    enum class Execution {
        Thread,  ///< Own thread; run()/update(). Must lock to touch mjData.
        Step,    ///< Simulation thread; beforeStep()/afterStep(). Already exclusive.
    };

    virtual ~MujocoRos2PluginInterface() = default;

    /** @brief Which execution model this plugin needs. Not URDF-overridable. */
    virtual Execution execution() const { return Execution::Thread; }

    /**
     * @brief What the simulation hands a plugin. Every member outlives the plugin.
     */
    struct Context {
        /// This plugin's own node, named after the `<mujoco_ros2_plugin>`, with
        /// use_sim_time set and already added to a non-realtime executor. Safe to
        /// declare parameters on and to create publishers, subscriptions,
        /// services and timers with.
        rclcpp::Node::SharedPtr node;
        /// The compiled model. Ids and addresses resolved from it stay valid.
        mjModel *mujoco_model = nullptr;
        /// Live simulation data, written by the simulation thread. Only ever
        /// read while holding @ref sim_mutex.
        mjData *mujoco_data = nullptr;
        /// Held by the simulation thread around mj_step(). Take it to read
        /// mjData, and hold it for as little as possible: the simulation blocks
        /// on it, so anything expensive belongs outside the critical section
        /// (mj_copyData() a snapshot and work on that instead).
        std::mutex *sim_mutex = nullptr;
        /// The outer lock, held by whatever is resetting, teleporting, or running
        /// a batch of steps. Take it -- before @ref sim_mutex, never after -- for
        /// work on the plugin's own thread or in a service callback that must not
        /// land in the middle of any of those.
        ///
        /// Never take it in beforeStep()/afterStep(): the caller already holds it,
        /// and it is not recursive, so that deadlocks the simulation.
        std::mutex *step_mutex = nullptr;
        /// Set when the simulation shuts down. run() has to return once it is true.
        const std::atomic<bool> *stop = nullptr;
    };

    /**
     * @brief Read the declaration, resolve MuJoCo ids, create publishers.
     *
     * Called once, on the simulation's setup thread, before run() starts. The
     * node is usable here: creating publishers and declaring parameters is
     * allowed, and both are already namespaced by the declaration name.
     *
     * @param context Handed to the plugin, and also stored for run() to use.
     * @param info    The `<mujoco_ros2_plugin>` element.
     * @return False to abort loading this plugin, after logging what was wrong
     *         with its declaration. No thread is started and the instance is
     *         dropped; the rest of the simulation still comes up.
     */
    virtual bool configure(const Context &context, const MujocoRos2PluginInfo &info) = 0;

    /**
     * @brief The plugin's own loop, run on a thread of its own.
     *
     * The default implementation paces itself on simulation time at rate() and
     * calls update() with @ref Context::sim_mutex held, which is what a plugin
     * that only copies values out of mjData wants.
     *
     * Override it when the plugin cannot work under that contract: rendering a
     * depth image or a lidar sweep takes far too long to hold the simulation
     * mutex for, so those take a snapshot instead --
     *
     * @code
     * { std::lock_guard<std::mutex> lock(*context_.sim_mutex);
     *   mj_copyData(snapshot_, context_.mujoco_model, context_.mujoco_data); }
     * @endcode
     *
     * -- and render from the snapshot outside the lock. An override must poll
     * `*context_.stop` and return promptly once it is set, or shutdown hangs.
     */
    virtual void run() {
        using namespace std::chrono_literals;

        while (rclcpp::ok() && !context_.stop->load()) {
            // Reading mjData::time unlocked is deliberate: it only decides when
            // the next sample is due, and the sample itself is taken under the
            // lock below.
            if (!due(context_.mujoco_data->time)) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            std::lock_guard<std::mutex> lock(*context_.sim_mutex);
            const rclcpp::Time stamp(
                static_cast<int64_t>(context_.mujoco_data->time * 1e9), RCL_ROS_TIME);
            update(context_.mujoco_data, stamp);
        }
    }

    /**
     * @brief Act on the state the coming mj_step() will integrate from.
     *
     * Execution::Step only, called on the simulation thread with the simulation
     * exclusively this plugin's -- no locking needed, and nothing else can be
     * mid-step. This is where inputs belong: `xfrc_applied`, `qfrc_applied`, or a
     * `ctrl` entry no controller claims. Writing every step is what makes those
     * fields mean what they look like, since MuJoCo does not clear them.
     *
     * Runs inside the physics loop, so it delays the step by its own duration:
     * keep it allocation-free and never block. Use due() to act at rate()
     * instead of on every step.
     */
    virtual void beforeStep(mjData *mujoco_data) { (void)mujoco_data; }

    /**
     * @brief Read the state mj_step() just produced.
     *
     * Execution::Step only, same contract as beforeStep(). The counterpart to
     * update() for plugins that observe rather than act, one call per physics
     * step rather than per rate() period -- decimate with due() when publishing.
     *
     * @param stamp Simulation time after the step, for the message header.
     */
    virtual void afterStep(const mjData *mujoco_data, const rclcpp::Time &stamp) {
        (void)mujoco_data;
        (void)stamp;
    }

    /**
     * @brief Take one sample. Called by the default run() with the simulation
     *        mutex held, so the simulation is blocked for its duration.
     *
     * Copy what is needed out of @p mujoco_data and publish. Publishing a small
     * message directly is fine here; anything that can block belongs on a
     * realtime_tools::RealtimePublisher, and anything expensive belongs in a
     * run() override working off a snapshot.
     *
     * @param mujoco_data Live simulation data, safe to read for this call only.
     * @param stamp       Simulation time of @p mujoco_data, for the message header.
     */
    virtual void update(const mjData *mujoco_data, const rclcpp::Time &stamp) {
        (void)mujoco_data;
        (void)stamp;
    }

    /**
     * @brief Ask for one more output covering the state after the current step.
     *
     * Deterministic stepping needs this: the StepSimulation service must not
     * answer before everything that publishes derived data has published it for
     * the final physics state, or a client reads output from a state it has
     * already stepped past. A plugin whose work happens on its own thread --
     * anything rendering -- is otherwise free to be mid-frame when the service
     * returns.
     *
     * Two-phase on purpose: the simulation asks every plugin first and only then
     * waits, so their frames are produced in parallel rather than one after
     * another. The returned value identifies the requested output and means
     * nothing outside this plugin.
     *
     * The default claims nothing is pending, which is right for a plugin that
     * publishes synchronously from update() or the step hooks: by the time the
     * service can observe it, its output is already out.
     */
    virtual std::uint64_t requestSynchronousFrame() { return 0; }

    /**
     * @brief Wait until the output identified by @p sequence has been published.
     *
     * @param sequence Value from a previous requestSynchronousFrame().
     * @param timeout  Give up after this long and return false.
     * @return True once it is out, false on timeout. False fails the step
     *         request, so do not return it for a plugin that simply has nothing
     *         to wait for.
     */
    virtual bool waitForSynchronousFrame(
            std::uint64_t sequence, std::chrono::milliseconds timeout) {
        (void)sequence;
        (void)timeout;
        return true;
    }

    /**
     * @brief Store the context, read the shared parameters, then configure().
     *
     * Called by MujocoRos2PluginLoader; not for plugins to override.
     */
    bool initialize(const Context &context, const MujocoRos2PluginInfo &info) {
        context_ = context;
        info_ = info;

        const auto rate = info.parameters.find("rate");
        if (rate != info.parameters.end() && !rate->second.empty()) {
            try {
                rate_ = std::stod(rate->second);
            } catch (const std::exception &) {
                RCLCPP_ERROR(context.node->get_logger(),
                    "Parameter 'rate' is not a number: '%s'", rate->second.c_str());
                return false;
            }
            if (rate_ <= 0.0) {
                RCLCPP_ERROR(context.node->get_logger(),
                    "Parameter 'rate' has to be positive, got %f", rate_);
                return false;
            }
            rate_declared_ = true;
        }

        return configure(context, info);
    }

    /** @brief The `<mujoco_ros2_plugin>` name; also this plugin's node name. */
    const std::string &name() const { return info_.name; }

    /** @brief Sampling rate in simulated Hz, from `<param name="rate">`. */
    double rate() const { return rate_; }

protected:
    MujocoRos2PluginInterface() = default;

    /**
     * @brief True once per 1/rate() of simulated time; false in between.
     *
     * The one place pacing is implemented, shared by both execution models: the
     * default run() calls it to decide when to sample, and an Execution::Step
     * plugin calls it to act at rate() rather than on every physics step.
     *
     * Paced on simulation time rather than wall time, so the rate holds under any
     * real_time_factor and stops advancing while the simulation is paused. The
     * first call is always due, a reset (time moving backwards) restarts pacing
     * from the new time, and falling more than a period behind resynchronises
     * rather than firing a burst to catch up.
     *
     * Not thread-safe, and does not need to be: a plugin is either threaded or
     * stepped, so only one thread ever calls it.
     *
     * @param sim_time `mjData::time` of the state being considered.
     */
    bool due(mjtNum sim_time) {
        const double period = 1.0 / rate_;

        if (!due_started_) {
            due_started_ = true;
            last_due_ = sim_time;
            return true;
        }
        if (sim_time < last_due_) {
            last_due_ = sim_time;
            return true;
        }
        if (sim_time - last_due_ < period) {
            return false;
        }
        last_due_ += period;
        if (sim_time - last_due_ >= period) {
            last_due_ = sim_time;
        }
        return true;
    }

    /**
     * @brief Propose a rate for plugins whose natural rate is not 100 Hz.
     *
     * Call it from configure(). It is ignored when the declaration set `rate`
     * explicitly, so a value in the URDF always wins over the plugin's own idea
     * of a sensible default.
     */
    void set_default_rate(double rate) {
        if (!rate_declared_ && rate > 0.0) {
            rate_ = rate;
        }
    }

    /// Set before configure() runs, so run() and update() can rely on it.
    Context context_;
    /// The declaration this instance was built from.
    MujocoRos2PluginInfo info_;

private:
    double rate_ = 100.0;
    /// due() pacing state.
    mjtNum last_due_ = 0;
    bool due_started_ = false;
    /// True once `<param name="rate">` has been applied, which locks out
    /// set_default_rate().
    bool rate_declared_ = false;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_PLUGIN_INTERFACE_HPP_
