#!/bin/env python3
"""Shared helpers for the per-robot example launch smoke tests.

Each example is brought up headless: the robot xacro is converted to an MJCF model
with ``xacro2mjcf.py`` and the ``mujoco_ros2_control`` node is started on it (with a
``robot_state_publisher`` feeding the robot_description so the controller manager can
load the MuJoCo hardware). The test then only checks that the node comes up, i.e.
MuJoCo accepted the generated model and ros2_control initialised - a malformed MJCF
or a broken xacro would make the node FATAL and never register on the graph.

The interactive MuJoCo viewer is never opened (``show_gui=False``) - these tests
have no display. What ``DISABLE_OPENGL`` controls is offscreen rendering: with it
unset or ``0`` the GL-backed sensors render through EGL, and with ``DISABLE_OPENGL=1``
they are left out so the tests run on a machine with no working GL at all. CI
runs with GL enabled (``DISABLE_OPENGL=0`` plus surfaceless EGL on llvmpipe), so
the camera and LiDAR paths are exercised there.

An example whose spec carries a ``terrain`` entry is brought up on a generated
stepping-stones scene instead of the flat ``mjcf/scene.xml``, matching what its
launch file does by default - so the shipped default is the configuration under
test.

An example is skipped (not failed) when its robot description package is not
installed, or when an asset group it needs was not downloaded (any of the
``DOWNLOAD_*`` CMake options turned off).
"""

import os
import subprocess

import launch
import launch_testing.actions
import xacro
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

PKG = "mujoco_ros2_control_examples"


def opengl_enabled():
    """GL toggle. Set DISABLE_OPENGL=1 to drop the sensors that need rendering."""
    return os.environ.get("DISABLE_OPENGL", "0") != "1"


# Per-robot configuration. ``requires`` lists ament packages that must be installed
# for the example to build a robot description; ``requires_files`` is a path - or a
# list of paths - under this package's share dir that must all exist, used for the
# assets fetched at configure time. ``assets_option`` names the CMake option that
# fetches them, so a skip says which switch to turn back on.
ROBOTS = {
    "franka": {
        "requires": ["franka_description"],
        # The description loads the D435 wrist mount and the IndustReal peg rows
        # unconditionally, so both downloads have to be present. Without this the
        # example does not skip with the assets off - it fails inside MuJoCo on a
        # <mesh> pointing at a file that was never fetched.
        "requires_files": [
            ("meshes", "franka", "realsense_d435", "RealSenseD435_Camera_Mount.obj"),
            ("meshes", "industreal", "pegs", "industreal_round_peg_8mm.obj"),
        ],
        "assets_option": "DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET",
        "xacro": ("urdf", "franka", "franka.urdf.xacro"),
        "controllers": ("config", "franka", "franka_controllers.yaml"),
        # The wrist camera's stream settings live in their own file, the way
        # franka.launch.py passes them; without it the D435 site is present but
        # no camera is configured, so a GUI run would exercise no rendering.
        "extra_configs": (("config", "franka", "realsense_d435.yaml"),),
        "mappings": {
            "name": "franka",
            "mujoco": "true",
            "arm_id": "fr3",
            "hand": "true",
            "ee_id": "franka_hand",
        },
        # xacro args this robot accepts beyond the static mappings above.
        "supports": ("headless",),
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
    "ur_multi": {
        "requires": ["ur_description"],
        "requires_files": None,
        "xacro": ("urdf", "ur", "ur_multi.urdf.xacro"),
        "controllers": ("config", "ur", "ur_multi_controllers.yaml"),
        "mappings": {
            "name": "ur_multi",
            # Three different arms, matching ur_multi.launch.py's defaults.
            "ur_types": "ur3e,ur5e,ur10e",
            "mujoco": "true",
        },
        "xacro2mjcf": {},
    },
    "unitree_h1": {
        "requires": [],
        # The upstream URDF is downloaded and patched at build time.
        "requires_files": ("urdf", "unitree_h1", "unitree_h1.urdf"),
        "assets_option": "DOWNLOAD_UNITREE_H1_ASSETS",
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
    "unitree_g1": {
        "requires": [],
        # The upstream URDF is downloaded and patched at build time.
        "requires_files": ("urdf", "unitree_g1", "unitree_g1.urdf"),
        "assets_option": "DOWNLOAD_UNITREE_G1_ASSETS",
        "xacro": ("urdf", "unitree_g1", "unitree_g1.urdf.xacro"),
        "controllers": ("config", "unitree_g1", "unitree_g1_controllers.yaml"),
        "mappings": {
            "name": "unitree_g1",
            "mujoco": "true",
        },
        "xacro2mjcf": {
            "base_link": "pelvis",
            "floating": True,
            "initial_position": "0 0 0.78",
            "initial_orientation": "0 0 0",
        },
        # unitree_g1.launch.py spawns on stepping stones by default; keep the
        # arguments here in step with the launch file's defaults.
        "terrain": ["--difficulty", "0.5", "--seed", "42", "--collider", "mesh",
                    "--field-half", "15.0", "--max-height-var", "0.10",
                    "--max-tilt-deg", "8.0", "--max-drop-frac", "0.15"],
    },
}


def _generate_terrain(robot, args):
    """Write a stepping-stones scene and return its path.

    Run synchronously while the launch description is being built, so the scene
    exists before xacro2mjcf starts and a generator failure surfaces here rather
    than as a MuJoCo load error. The output goes beside the model directory, not
    inside it - xacro2mjcf wipes its own working directory on startup.
    """
    out_dir = f"/tmp/mujoco_example_{robot}_terrain"
    subprocess.run(
        [os.path.join(get_package_prefix("mujoco_ros2_control"),
                      "lib", "mujoco_ros2_control", "prepare_terrain.py"),
         "--out-dir", out_dir, *args],
        check=True,
    )
    return os.path.join(out_dir, "terrain_scene.xml")


def availability(robot):
    """Return (ok, reason). ok=False means the test should skip this example."""
    spec = ROBOTS[robot]
    for dep in spec["requires"]:
        try:
            get_package_share_directory(dep)
        except PackageNotFoundError:
            return False, f"{dep} is not installed"
    share = get_package_share_directory(PKG)
    required = spec["requires_files"] or ()
    # A bare tuple of path components is one path; a list holds several.
    if required and not isinstance(required, list):
        required = [required]
    for rf in required:
        if not os.path.exists(os.path.join(share, *rf)):
            option = spec.get("assets_option")
            hint = f" (build with -D{option}=ON)" if option else ""
            return False, f"missing {os.path.join(*rf)}{hint}"
    return True, ""


def make_skipped_description():
    """The launch description to use for an example that cannot run here.

    launch_testing needs the launch to stay alive until the test cases have run.
    A description holding only ``ReadyToTest()`` has no processes, so the launch
    ends at once and the run reports "Launch stopped before the active tests
    finished" - a failure - instead of the skip the test intended. A trivial
    sleeping process keeps the launch open long enough for the skipped cases to
    be collected; launch_testing tears it down as soon as they have.
    """
    return launch.LaunchDescription([
        ExecuteProcess(cmd=["sleep", "30"]),
        launch_testing.actions.ReadyToTest(),
    ])


def make_test_description(robot):
    """Build the LaunchDescription that brings the given example up headless."""
    spec = ROBOTS[robot]
    share = get_package_share_directory(PKG)
    model_path = f"/tmp/mujoco_example_{robot}"
    model_file = os.path.join(model_path, "main.xml")

    xacro_file = os.path.join(share, *spec["xacro"])
    mappings = dict(spec["mappings"])
    # The franka model's only GL camera is the D435 wrist camera; its `headless`
    # argument turns that rendering off so the node never tries to create an
    # OpenGL context. CI leaves it on, so the camera is exercised there; the
    # mount and camera stay in the description either way.
    if "headless" in spec.get("supports", ()):
        mappings["headless"] = "false" if opengl_enabled() else "true"
    robot_description = {
        "robot_description": xacro.process_file(
            xacro_file, mappings=mappings
        ).toprettyxml(indent="  ")
    }

    config_files = [os.path.join(share, *spec["controllers"])]
    for extra in spec.get("extra_configs", ()):
        config_files.append(os.path.join(share, *extra))
    if spec.get("terrain"):
        scene_file = _generate_terrain(robot, spec["terrain"])
    else:
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
            *config_files,
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
