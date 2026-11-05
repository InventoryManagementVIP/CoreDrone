"""SITL smoke test (`just sim-smoke`, CI on every PR touching drone/ sim/ deps/ docker/).

Gazebo Harmonic + ArduPilot SITL (GPS disabled, external navigation) + the full stack with
localization:=groundtruth. mission:=smoke flies arm -> take off 1.5 m -> hover 10 s -> land, entirely
through inv_flight_interface's actions. The test only observes; it never talks to the FC.
"""

from __future__ import annotations

import math
import os
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from inv_interfaces.msg import FlightState, MissionStatus

TAKEOFF_ALT_M = 1.5
SPAWN_Z_M = 0.195  # base_link height on the ground (sim/worldgen/drone_model.py)
# Wall clock. ~3 min on a 16-core dev machine; nested inside `just sim-smoke` (timeout 900) and the
# CI step (20 min).
TIMEOUT_S = float(os.environ.get("SIM_SMOKE_TIMEOUT_S", "720"))
RUN_DIR = tempfile.mkdtemp(prefix="sim_smoke_")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    share = get_package_share_directory("inv_bringup")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "drone.launch.py")),
                launch_arguments={
                    "platform": "sim",
                    "localization": "groundtruth",
                    "mapping": "none",
                    "perception": "none",
                    "sensors": "none",  # no cameras: CI runners have no GPU
                    "headless": "true",
                    "mission": "smoke",
                    "run_dir": RUN_DIR,
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSmokeFlight(unittest.TestCase):
    def test_takeoff_hover_land(self):
        rclpy.init()
        node = rclpy.create_node("sim_smoke_observer")
        status: list[MissionStatus] = []
        flight: list[FlightState] = []
        hover_truth: list[tuple[float, float, float]] = []
        max_z = [0.0]
        last_status = [None]
        ap_poses = [0]

        def on_status(msg):
            status.append(msg)
            last_status[0] = msg

        def on_truth(msg):
            p = msg.pose.pose.position
            max_z[0] = max(max_z[0], p.z)
            s = last_status[0]
            if s is not None and s.state == MissionStatus.STATE_RUNNING and s.message == "hover":
                hover_truth.append((p.x, p.y, p.z))

        node.create_subscription(MissionStatus, "/mission/status", on_status, 10)
        node.create_subscription(FlightState, "/flight/state", flight.append, 10)
        node.create_subscription(Odometry, "/sim/ground_truth/odom", on_truth, qos_profile_sensor_data)
        # DoD: the FC's own estimate is available and stable (observed only, never used for control).
        node.create_subscription(
            PoseStamped,
            "/ap/pose/filtered",
            lambda _m: ap_poses.__setitem__(0, ap_poses[0] + 1),
            qos_profile_sensor_data,
        )
        terminal = {MissionStatus.STATE_COMPLETED, MissionStatus.STATE_FAILED, MissionStatus.STATE_ABORTED}
        deadline = time.monotonic() + TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if last_status[0] is not None and last_status[0].state in terminal:
                    break
            final = last_status[0]
            self.assertIsNotNone(final, "no /mission/status")
            self.assertEqual(final.state, MissionStatus.STATE_COMPLETED, f"mission ended: {final.message}")
            self.assertGreater(ap_poses[0], 0, "no /ap/pose/filtered from the flight controller")
            # Climbed to the commanded height (base_link starts SPAWN_Z_M above the floor).
            self.assertGreater(max_z[0], SPAWN_Z_M + TAKEOFF_ALT_M - 0.25, f"max height {max_z[0]:.2f} m")
            # Hovered in place: little drift and a steady height.
            self.assertGreater(len(hover_truth), 50, "too few ground-truth samples while hovering")
            xs, ys, zs = zip(*hover_truth, strict=True)
            drift = math.dist((xs[0], ys[0]), (xs[-1], ys[-1]))
            self.assertLess(drift, 0.3, f"horizontal drift while hovering {drift:.2f} m")
            self.assertLess(max(zs) - min(zs), 0.3, f"height varied by {max(zs) - min(zs):.2f} m while hovering")
            # Landed and disarmed.
            end = time.monotonic() + 30.0
            while time.monotonic() < end and (not flight or flight[-1].armed):
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertFalse(flight[-1].armed, "still armed after the mission")
        finally:
            node.destroy_node()
            rclpy.shutdown()
