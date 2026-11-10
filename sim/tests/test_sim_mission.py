"""Scan mission end to end (`just sim-mission`, nightly CI).

Flies the generated route on a warehouse world with the label camera rendered, decodes QR codes
and AprilTags with the CPU path, writes the SQLite queue and CSV, then scores the run against
ground_truth.json with sim/evaluation. Low scores are fine for now: the pipeline must run.

  launch_test sim/tests/test_sim_mission.py warehouse:=sim_small
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from inv_interfaces.msg import MissionStatus
from inv_interfaces.srv import ExportCsv

ROOT = Path(os.environ.get("INV_ROOT", Path(__file__).resolve().parents[2]))
RUN_DIR = ROOT / "runs" / "latest"
# Wall-clock budget. The sim_small route takes ~40 min at the ~0.2x real time of a CPU-only machine;
# nested inside `just sim-mission` (timeout 6000) and the nightly CI step (120 min).
TIMEOUT_S = float(os.environ.get("SIM_MISSION_TIMEOUT_S", "5400"))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    for stale in ("inventory.sqlite", "inventory.sqlite-wal", "inventory.sqlite-shm", "evaluation.json"):
        (RUN_DIR / stale).unlink(missing_ok=True)
    share = get_package_share_directory("inv_bringup")
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("warehouse", default_value="sim_small"),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "drone.launch.py")),
                launch_arguments={
                    "platform": "sim",
                    "localization": "groundtruth",
                    "mapping": "none",
                    "perception": "cpu",
                    "sensors": "label",
                    "headless": "true",
                    "mission": "scan",
                    "warehouse": LaunchConfiguration("warehouse"),
                    "run_dir": str(RUN_DIR),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestScanMission(unittest.TestCase):
    def test_scan_mission_and_evaluation(self):
        rclpy.init()
        node = rclpy.create_node("sim_mission_observer")
        last = [None]
        node.create_subscription(MissionStatus, "/mission/status", lambda m: last.__setitem__(0, m), 10)
        terminal = {MissionStatus.STATE_COMPLETED, MissionStatus.STATE_FAILED, MissionStatus.STATE_ABORTED}
        deadline = time.monotonic() + TIMEOUT_S
        progress = -1
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                s = last[0]
                if s is not None and s.waypoint_index != progress:
                    progress = s.waypoint_index
                    print(f"[sim_mission] waypoint {s.waypoint_index}/{s.waypoint_count} {s.message}", flush=True)
                if s is not None and s.state in terminal:
                    break
            final = last[0]
            self.assertIsNotNone(final, "no /mission/status")
            self.assertEqual(final.state, MissionStatus.STATE_COMPLETED, f"mission ended: {final.message}")

            client = node.create_client(ExportCsv, "/uplink/export_csv")
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
            future = client.call_async(
                ExportCsv.Request(filename=f"{final.mission_id}.csv", mission_id=final.mission_id)
            )
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            self.assertTrue(future.result() and future.result().success, "CSV export failed")

            report = subprocess.run(
                [sys.executable, "-m", "evaluation", "--run-dir", str(RUN_DIR), "--world", final.warehouse_id],
                cwd=ROOT / "sim",
                capture_output=True,
                text=True,
                check=False,
            )
            print(report.stdout, flush=True)
            self.assertEqual(report.returncode, 0, report.stderr)
            result = json.loads((RUN_DIR / "evaluation.json").read_text())
            # The pipeline must run end to end and read something; scores are tracked, not gated.
            self.assertGreater(result["items_read"], 0, "no item was decoded")
        finally:
            node.destroy_node()
            rclpy.shutdown()
