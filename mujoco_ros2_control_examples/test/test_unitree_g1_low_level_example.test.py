#!/bin/env python3
"""Launch test for the G1 driven over unitree_hg/LowCmd by UnitreeHgLowLevel.

Unlike the other example tests, this one does more than check that the node comes
up: the whole point of the plugin is that a LowCmd turns into torque and shows up
again in LowState, and nothing in ros2_control commands these joints, so the
round trip is the only thing that proves the path works.

First it checks that the robot holds its start pose with no LowCmd from the test
at all -- that is the 50 Hz hold published alongside the simulation, doing the work
the plugin deliberately does not do. Then it sends the arms somewhere else and
reads the measured positions back out of LowState: the legs keep the robot standing
while the elbows and a shoulder swing to a target well outside any tolerance the
start pose would satisfy. That second phase also covers the handover, since this
test then commands the same joints the hold does. Skips itself when unitree_hg is
not in the workspace (see .repositories).
"""

import os
import sys
import time
import unittest

import pytest
import rclpy
import rclpy.node

# example_smoke.py lives next to this test; make sure it is importable regardless
# of how the launch_test runner sets up sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_smoke import (  # noqa: E402
    availability,
    make_skipped_description,
    make_test_description,
)

ROBOT = "unitree_g1_low_level"
_AVAILABLE, _SKIP_REASON = availability(ROBOT)

try:
    from unitree_hg.msg import LowCmd, LowState
except ImportError:  # pragma: no cover - the availability check skips this too
    LowCmd = LowState = None
    _AVAILABLE, _SKIP_REASON = False, "unitree_hg is not installed"

# (motor index, joint, kp, kd, target) per driven joint, in LowCmd motor order --
# the order the joints are declared in urdf/unitree_g1/ros2_control.urdf.xacro,
# with that file's gains and initial positions. Holding this pose is what the
# position controllers do in the default example; here the plugin does it.
POSE = [
    (0, "left_hip_pitch_joint", 500.0, 2.0, -0.1),
    (1, "left_hip_roll_joint", 500.0, 2.0, 0.0),
    (2, "left_hip_yaw_joint", 500.0, 2.0, 0.0),
    (3, "left_knee_joint", 750.0, 4.0, 0.3),
    (4, "left_ankle_pitch_joint", 200.0, 2.0, -0.2),
    (5, "left_ankle_roll_joint", 200.0, 2.0, 0.0),
    (6, "right_hip_pitch_joint", 500.0, 2.0, -0.1),
    (7, "right_hip_roll_joint", 500.0, 2.0, 0.0),
    (8, "right_hip_yaw_joint", 500.0, 2.0, 0.0),
    (9, "right_knee_joint", 750.0, 4.0, 0.3),
    (10, "right_ankle_pitch_joint", 200.0, 2.0, -0.2),
    (11, "right_ankle_roll_joint", 200.0, 2.0, 0.0),
    (12, "waist_yaw_joint", 1000.0, 5.0, 0.0),
    (13, "waist_roll_joint", 200.0, 5.0, 0.0),
    (14, "waist_pitch_joint", 200.0, 5.0, 0.0),
    (15, "left_shoulder_pitch_joint", 200.0, 1.0, 0.3),
    (16, "left_shoulder_roll_joint", 200.0, 1.0, 0.25),
    (17, "left_shoulder_yaw_joint", 200.0, 1.0, 0.0),
    (18, "left_elbow_joint", 200.0, 1.0, 0.97),
    (19, "left_wrist_roll_joint", 200.0, 1.0, 0.15),
    (20, "left_wrist_pitch_joint", 200.0, 1.0, 0.0),
    (21, "left_wrist_yaw_joint", 200.0, 1.0, 0.0),
    (22, "right_shoulder_pitch_joint", 200.0, 1.0, 0.3),
    (23, "right_shoulder_roll_joint", 200.0, 1.0, -0.25),
    (24, "right_shoulder_yaw_joint", 200.0, 1.0, 0.0),
    (25, "right_elbow_joint", 200.0, 1.0, 0.97),
    (26, "right_wrist_roll_joint", 200.0, 1.0, -0.15),
    (27, "right_wrist_pitch_joint", 200.0, 1.0, 0.0),
    (28, "right_wrist_yaw_joint", 200.0, 1.0, 0.0),
]

# Where the arms are asked to go instead of their start pose, by motor index.
# Each is far enough from the start pose that reaching it can only be the
# plugin's torque: the elbows swing 0.67 rad, the left shoulder 0.6 rad. The
# legs and waist keep holding the start pose, so the robot stays standing while
# the arms move.
REACH = {18: 0.3, 25: 0.3, 15: 0.9}
# Read back: the three moved joints, plus a knee to show the legs are driven too.
TRACKED = (3, 15, 18, 25)
# The arms settle far inside this; the slack is for the knee, which also carries
# the robot's weight while the feet settle.
TOLERANCE = 0.1
# How long the robot is left with no command at all, to show that the hold is what
# keeps it standing. Long enough that slack joints would have put it on the floor.
HOLD_SECONDS = 3.0


@pytest.mark.launch_test
def generate_test_description():
    if not _AVAILABLE:
        return make_skipped_description()
    return make_test_description(ROBOT)


class TestUnitreeG1LowLevelExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("test_unitree_g1_low_level_example_node")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _command(reach):
        """One LowCmd holding the start pose, with `reach` overriding targets."""
        command = LowCmd()
        for index, _joint, kp, kd, target in POSE:
            motor = command.motor_cmd[index]
            motor.mode = 1
            motor.q = float(reach.get(index, target))
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(kp)
            motor.kd = float(kd)
        return command

    def _wait_for_pose(self, publisher, states, command, reach, timeout):
        """Publish `command` until every tracked joint is at its target.

        Returns the LowState that satisfied it. Publishing runs throughout rather
        than once up front, because that is what a policy does and because it is
        the stream, not one message, that the plugin reads on every physics step.
        """
        errors = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            publisher.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if not states:
                continue
            state = states[-1]
            errors = {
                index: abs(state.motor_state[index].q - reach.get(index, target))
                for index, _joint, _kp, _kd, target in POSE
                if index in TRACKED
            }
            if all(error < TOLERANCE for error in errors.values()):
                return state
        self.fail(
            "the commanded joints never reached their targets: "
            + ", ".join(
                f"{joint} off by {errors[index]:.3f} rad"
                for index, joint, _kp, _kd, _target in POSE
                if index in TRACKED
            )
            if errors
            else "no unitree_hg/LowState was published on /lowstate"
        )

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_lowcmd_moves_the_joints_and_lowstate_reports_it(self):
        # The mujoco node only registers once it has loaded the merged MJCF; if
        # MuJoCo rejected the model (or the xacro failed) it FATALs and never appears.
        deadline = time.time() + 30.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if "mujoco_ros2_control" in self.node.get_node_names():
                break
        else:
            self.fail(
                "mujoco_ros2_control node did not come up; MuJoCo failed to load "
                "the unitree_g1 low-level example model"
            )

        states = []
        subscription = self.node.create_subscription(
            LowState, "/lowstate", states.append, 10
        )
        self.addCleanup(self.node.destroy_subscription, subscription)

        # LowState is published by the plugin, not by a controller, so its arrival
        # already means the plugin configured and is being stepped.
        deadline = time.time() + 30.0
        while time.time() < deadline and not states:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(states, "no unitree_hg/LowState was published on /lowstate")

        # Nothing has been commanded by this test, and nothing will be for the next
        # few seconds: what keeps the robot standing here is the hold publisher. A
        # robot with slack joints has left the start pose (and the floor) long
        # before this window is over.
        deadline = time.time() + HOLD_SECONDS
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        held = states[-1]
        for index, joint, _kp, _kd, target in POSE:
            if index not in TRACKED:
                continue
            self.assertAlmostEqual(
                held.motor_state[index].q, target, delta=TOLERANCE,
                msg=f"{joint} left the start pose while only the hold publisher was "
                    f"commanding it",
            )
        upright = abs(held.imu_state.quaternion[0])
        self.assertGreater(
            upright, 0.9,
            f"the pelvis is not upright after {HOLD_SECONDS} s of holding: "
            f"quaternion {held.imu_state.quaternion}",
        )
        # A stiff PD holding a humanoid up cannot be reporting zero torque; zero
        # here would mean LowState is filled from somewhere other than the joint,
        # or that the hold is not reaching the plugin at all.
        self.assertGreater(
            abs(held.motor_state[3].tau_est), 0.1,
            "left_knee_joint reports no torque while holding the robot up",
        )
        start = {index: held.motor_state[index].q for index in TRACKED}

        # Only now does this test become a publisher on /lowcmd, so the hold is
        # demonstrably the only thing that commanded the joints in the window above.
        publisher = self.node.create_publisher(LowCmd, "/lowcmd", 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        # Now ask the arms somewhere else. The hold keeps publishing the start pose
        # throughout, so these targets have to win on rate: the loop below spins on
        # a 500 Hz LowState and so publishes far faster than the hold's 50 Hz, and
        # the plugin applies whatever arrived last.
        reached = self._wait_for_pose(publisher, states, self._command(REACH), REACH, 40.0)
        for index, joint, _kp, _kd, _target in POSE:
            if index not in REACH:
                continue
            travelled = abs(reached.motor_state[index].q - start[index])
            self.assertGreater(
                travelled, 0.3,
                f"{joint} reached its target without moving: it was already there, "
                f"so nothing proves the LowCmd produced any torque",
            )

        # The IMU is read straight out of mjData::sensordata, and a zero
        # quaternion would mean the sensors were never bound.
        norm = sum(component**2 for component in reached.imu_state.quaternion) ** 0.5
        self.assertAlmostEqual(
            norm, 1.0, delta=1e-3,
            msg=f"LowState carries no valid IMU orientation: {reached.imu_state.quaternion}",
        )
