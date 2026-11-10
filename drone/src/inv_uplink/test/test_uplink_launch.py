"""launch_testing: the uplink starts, publishes diagnostics and exports a CSV on request."""

import datetime as dt
import os
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch.launch_description_sources import PythonLaunchDescriptionSource

from inv_interfaces.srv import ExportCsv
from inv_inventory.queue import ScanQueue
from inv_inventory.records import make_scan_record

TMP = tempfile.mkdtemp(prefix="uplink_test_")
DB = os.path.join(TMP, "inventory.sqlite")


@pytest.mark.launch_test
def generate_test_description():
    q = ScanQueue(DB)
    q.put(
        make_scan_record(
            mission_id="m1",
            drone_id="d1",
            warehouse_id="sim_small",
            timestamp=dt.datetime.now(dt.timezone.utc),
            code="SKU-1",
            symbology="qr",
            slot_code=None,
            rack_id=None,
            level=None,
            position_map=None,
            confidence=1.0,
            attribution_method="none",
        )
    )
    q.close()
    share = get_package_share_directory("inv_uplink")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "uplink.launch.py")),
                launch_arguments={"db_path": DB, "export_dir": os.path.join(TMP, "exports")}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestUplink(unittest.TestCase):
    def test_syncs_and_exports(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_uplink_client")
            diags = []
            node.create_subscription(DiagnosticArray, "/diagnostics", diags.append, 10)
            client = node.create_client(ExportCsv, "/uplink/export_csv")
            self.assertTrue(client.wait_for_service(timeout_sec=20.0))
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                pending = [kv.value for d in diags for s in d.status for kv in s.values if kv.key == "pending"]
                if pending and pending[-1] == "0":
                    break
            self.assertTrue(diags, "no /diagnostics")
            outside = os.path.join(TMP, "outside.csv")
            future = client.call_async(ExportCsv.Request(filename=outside))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            self.assertFalse(future.result().success)
            self.assertFalse(os.path.exists(outside))
            future = client.call_async(ExportCsv.Request(filename="out.csv"))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            self.assertTrue(future.result().success)
            self.assertEqual(future.result().count, 1)
            self.assertTrue(os.path.exists(os.path.join(TMP, "exports", "out.csv")))
            node.destroy_node()
        finally:
            rclpy.shutdown()
