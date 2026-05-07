#!/bin/env python3
"""No-GL unit tests for the test_data sensor wiring.

These never start the mujoco_ros2_control node, so they don't need GL or
even ROS to be running. They verify that the test xacro and YAML files
declare every sensor the launch test asserts on, and that the URDF still
generates a valid MJCF. The launch test (simple_launch.test.py) covers
end-to-end behavior; this file catches regressions in the static wiring.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml


def _test_data_dir():
    """Resolve test_data/ from the installed share dir if available, else
    fall back to the source-tree path next to this file (for ad-hoc
    pre-install runs)."""
    try:
        from ament_index_python import get_package_share_directory
        return os.path.join(
            get_package_share_directory("mujoco_ros2_control"), "test_data")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "data")


class TestXacroSensorWiring(unittest.TestCase):
    """Parse the xacro and assert all expected sensor declarations exist."""

    @classmethod
    def setUpClass(cls):
        xacro_path = os.path.join(_test_data_dir(), "double_pendulum.urdf.xacro")
        cls.urdf_str = xacro.process_file(
            xacro_path, mappings={"command_mode": "effort"}
        ).toprettyxml(indent="  ")
        cls.root = ET.fromstring(cls.urdf_str)

    def _ros2_control_sensors(self):
        return {
            s.get("name"): s
            for r in self.root.iter("ros2_control")
            for s in r.findall("sensor")
        }

    def _mujoco_blocks(self):
        return list(self.root.iter("mujoco"))

    def test_imu_sensor_declared(self):
        sensors = self._ros2_control_sensors()
        self.assertIn("link3_imu", sensors)
        ifaces = {si.get("name") for si in sensors["link3_imu"].findall("state_interface")}
        for axis in ("x", "y", "z"):
            self.assertIn(f"angular_velocity.{axis}", ifaces)
            self.assertIn(f"linear_acceleration.{axis}", ifaces)
        for comp in ("x", "y", "z", "w"):
            self.assertIn(f"orientation.{comp}", ifaces)

    def test_wrench_sensor_declared(self):
        sensors = self._ros2_control_sensors()
        self.assertIn("link3_wrench", sensors)
        ifaces = {si.get("name") for si in sensors["link3_wrench"].findall("state_interface")}
        for axis in ("x", "y", "z"):
            self.assertIn(f"force.{axis}", ifaces)
            self.assertIn(f"torque.{axis}", ifaces)

    def test_pose_sensor_declared(self):
        sensors = self._ros2_control_sensors()
        self.assertIn("link3_pose", sensors)
        ifaces = {si.get("name") for si in sensors["link3_pose"].findall("state_interface")}
        for axis in ("x", "y", "z"):
            self.assertIn(f"position.{axis}", ifaces)
        for comp in ("x", "y", "z", "w"):
            self.assertIn(f"orientation.{comp}", ifaces)

    def test_mujoco_camera_present(self):
        cams = [c for blk in self._mujoco_blocks() for c in blk.iter("camera")]
        names = {c.get("name") for c in cams}
        self.assertIn("test_camera", names)

    def test_lidar_sites_present(self):
        # Sites whose name starts with the configured site_prefix ("lidar_")
        # become MujocoGLLidar instances at runtime.
        sites = [s for blk in self._mujoco_blocks() for s in blk.iter("site")]
        names = {s.get("name") for s in sites}
        self.assertIn("lidar_scan", names)
        self.assertIn("lidar_cloud", names)

    def test_mujoco_native_sensors_present(self):
        # The plugin-side sensor matching needs accelerometer/gyro/force/torque
        # plus a framepos/framequat at body=link3.
        native = [s for blk in self._mujoco_blocks() for s in list(blk)]
        types_at_link3 = set()
        for el in native:
            if el.tag != "sensor":
                continue
            for child in el:
                if child.get("site") == "link3" or child.get("objname") == "link3":
                    types_at_link3.add(child.tag)
        for needed in ("accelerometer", "gyro", "force", "torque", "framepos", "framequat"):
            self.assertIn(needed, types_at_link3, f"missing MuJoCo {needed} at link3")


class TestControllersYaml(unittest.TestCase):
    """The launch test spawns these broadcasters by name; if the YAML drifts
    the spawner fails with a confusing 'unknown controller' error."""

    @classmethod
    def setUpClass(cls):
        path = os.path.join(_test_data_dir(), "double_pendulum_controllers.yaml")
        with open(path, "r") as f:
            cls.cfg = yaml.safe_load(f)

    def test_broadcasters_registered(self):
        cm = self.cfg["controller_manager"]["ros__parameters"]
        self.assertEqual(cm["link3_imu_sensor"]["type"],
                         "imu_sensor_broadcaster/IMUSensorBroadcaster")
        self.assertEqual(cm["link3_wrench_sensor"]["type"],
                         "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster")
        self.assertEqual(cm["link3_pose_sensor"]["type"],
                         "pose_broadcaster/PoseBroadcaster")

    def test_broadcaster_params_match_urdf_sensor_names(self):
        # The broadcaster's sensor_name must match a <sensor name> in the URDF
        # — otherwise it won't find any state interfaces to read.
        self.assertEqual(self.cfg["link3_imu_sensor"]["ros__parameters"]["sensor_name"],
                         "link3_imu")
        self.assertEqual(self.cfg["link3_wrench_sensor"]["ros__parameters"]["sensor_name"],
                         "link3_wrench")
        self.assertEqual(self.cfg["link3_pose_sensor"]["ros__parameters"]["pose_name"],
                         "link3_pose")


class TestLidarParamsYaml(unittest.TestCase):
    """The two lidar sites must be configured for different output kinds —
    that's the whole point of having both sites."""

    @classmethod
    def setUpClass(cls):
        path = os.path.join(_test_data_dir(), "double_pendulum_lidar_params.yaml")
        with open(path, "r") as f:
            cls.cfg = yaml.safe_load(f)

    def test_scan_mode(self):
        self.assertEqual(self.cfg["lidar_scan"]["ros__parameters"]["output"], "scan")

    def test_cloud_mode(self):
        self.assertEqual(self.cfg["lidar_cloud"]["ros__parameters"]["output"], "cloud")


if __name__ == "__main__":
    unittest.main()
