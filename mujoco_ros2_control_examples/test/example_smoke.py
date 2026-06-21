#!/bin/env python3
"""Shared helpers for the per-robot example launch smoke tests.

Each example is brought up headless: the robot xacro is converted to an MJCF model
with ``xacro2mjcf.py`` and the ``mujoco_ros2_control`` node is started on it (with a
``robot_state_publisher`` feeding the robot_description so the controller manager can
load the MuJoCo hardware). The test then only checks that the node comes up, i.e.
MuJoCo accepted the generated model and ros2_control initialised - a malformed MJCF
or a broken xacro would make the node FATAL and never register on the graph.

OpenGL is optional: with ``DISABLE_OPENGL=1`` the simulation runs fully headless
(``show_gui=False``); otherwise the GUI is shown, matching the other launch tests in
this repository. CI is expected to export ``DISABLE_OPENGL=1``.

An example is skipped (not failed) when its robot description package is not
installed, or - for Unitree H1 - when the upstream assets were not downloaded
(``-DDOWNLOAD_UNITREE_H1_ASSETS=OFF``).
"""

import os

import launch
import launch_testing.actions
import xacro
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

PKG = "mujoco_ros2_control_examples"


def opengl_enabled():
    """Headless toggle. Set DISABLE_OPENGL=1 in the env to run without a GUI."""
    return os.environ.get("DISABLE_OPENGL", "0") != "1"


# Per-robot configuration. ``requires`` lists ament packages that must be installed
# for the example to build a robot description; ``requires_files`` is a path under
# this package's share dir that must exist (used for the downloaded Unitree assets).
ROBOTS = {
    "franka": {
        "requires": ["franka_description"],
        "requires_files": None,
        "xacro": ("urdf", "franka", "franka.urdf.xacro"),
        "controllers": ("config", "franka", "franka_controllers.yaml"),
        "mappings": {
            "name": "franka",
            "mujoco": "true",
            "arm_id": "fr3",
            "hand": "true",
            "ee_id": "franka_hand",
        },
        "xacro2mjcf": {},
    },
    "ur": {
        "requires": ["ur_description"],
        "requires_files": None,
        "xacro": ("urdf", "ur", "ur.urdf.xacro"),
        "controllers": ("config", "ur", "ur_controllers.yaml"),
        "mappings": {
            "name": "ur",
            "ur_type": "ur5e",
            "mujoco": "true",
        },
        "xacro2mjcf": {},
    },
    "unitree_h1": {
        "requires": [],
        # The upstream URDF is downloaded and patched at build time.
        "requires_files": ("urdf", "unitree_h1", "unitree_h1.urdf"),
        "xacro": ("urdf", "unitree_h1", "unitree_h1.urdf.xacro"),
        "controllers": ("config", "unitree_h1", "unitree_h1_controllers.yaml"),
        "mappings": {
            "name": "unitree_h1",
            "mujoco": "true",
            "mujoco_effort": "false",
        },
        "xacro2mjcf": {
            "base_link": "pelvis",
            "floating": True,
            "initial_position": "0 0 1.05",
            "initial_orientation": "0 0 0",
        },
    },
}


def availability(robot):
    """Return (ok, reason). ok=False means the test should skip this example."""
    spec = ROBOTS[robot]
    for dep in spec["requires"]:
        try:
            get_package_share_directory(dep)
        except PackageNotFoundError:
            return False, f"{dep} is not installed"
    share = get_package_share_directory(PKG)
    rf = spec["requires_files"]
    if rf and not os.path.exists(os.path.join(share, *rf)):
        return False, (
            f"missing {os.path.join(*rf)} "
            "(build with -DDOWNLOAD_UNITREE_H1_ASSETS=ON)"
        )
    return True, ""


def make_test_description(robot):
    """Build the LaunchDescription that brings the given example up headless."""
    spec = ROBOTS[robot]
    share = get_package_share_directory(PKG)
    model_path = f"/tmp/mujoco_example_{robot}"
    model_file = os.path.join(model_path, "main.xml")

    xacro_file = os.path.join(share, *spec["xacro"])
    robot_description = {
        "robot_description": xacro.process_file(
            xacro_file, mappings=spec["mappings"]
        ).toprettyxml(indent="  ")
    }

    controllers_file = os.path.join(share, *spec["controllers"])
    scene_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"
    )

    xacro2mjcf_params = [
        {"robot_descriptions": [robot_description["robot_description"]]},
        {"input_files": [scene_file]},
        {"output_file": model_file},
        {"mujoco_files_path": model_path},
    ]
    for key, value in spec["xacro2mjcf"].items():
        xacro2mjcf_params.append({key: value})

    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=xacro2mjcf_params,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers_file,
            {"simulation_frequency": 200.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": model_file},
            {"show_gui": False},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher,
            xacro2mjcf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=xacro2mjcf,
                    on_exit=[
                        mujoco,
                        TimerAction(
                            period=5.0,
                            actions=[launch_testing.actions.ReadyToTest()],
                        ),
                    ],
                )
            ),
        ]
    )
