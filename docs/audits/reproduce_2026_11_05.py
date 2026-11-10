"""Reproduce audit observations without a flight controller or network access.

Run inside the built CoreDrone ROS environment. These assertions describe the
audited behavior, not the desired behavior; they should change after remediation.
All file writes are confined to a TemporaryDirectory.
"""

from __future__ import annotations

import copy
import csv
import json
import math
import threading
import uuid
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS

import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.action import GoalResponse

from inv_flight_interface.backend import FcState
from inv_flight_interface.control import Box, Limits, Setpoint
from inv_flight_interface.node import DEFAULTS, Command, FlightInterfaceNode, Pose
from inv_interfaces.msg import LocalizationStatus, MissionStatus
from inv_inventory.queue import ScanQueue
from inv_layout import parse_layout
from inv_localization.adapter import classify
from inv_safety.monitor import SafetyMonitor
from inv_safety.policy import SafetyConfig
from inv_uplink.backends import MockBackend
from inv_uplink.export import export_csv
from inv_uplink.node import UplinkNode

ROOT = Path(__file__).resolve().parents[2]


class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


def observed(name, value):
    print(f"CONFIRMED {name}: {value}", flush=True)


def main():
    # Use ROS's real parameter parser with the same file ordering as bringup.
    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(ROOT / "drone/src/inv_bringup/config/real.yaml"),
            "--params-file",
            str(ROOT / "drone/src/inv_flight_interface/config/flight_interface.yaml"),
        ]
    )
    node = FlightInterfaceNode()
    try:
        actual = (node.limits.max_horizontal_speed, node.limits.max_vertical_speed)
        assert actual == (1.0, 0.5)
        observed("real speed overrides lost", actual)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    logger = Logger()
    sent = []
    fake = NS(
        p={**DEFAULTS},
        limits=Limits(),
        fence=Box((-1.0, -1.0, 0.0), (1.0, 1.0, 5.0)),
        _last_setpoint=Setpoint(1.0, 0.0, 0.0),
        alignment=NS(rotation=0.0),
        _map_to_odom_yaw=lambda: 0.0,
        backend=NS(send_velocity=lambda *v: sent.append(v)),
    )
    fake._limit_speeds = lambda *a: FlightInterfaceNode._limit_speeds(fake, *a)
    FlightInterfaceNode._send(fake, Setpoint(1.0, 0.0, 0.0), Pose(1.0, 0.0, 1.0, 0.0), 0.05)
    assert sent[-1][0] > 0.0
    observed("outward velocity at fence after slew", sent[-1][0])

    FlightInterfaceNode._send(fake, Setpoint(math.nan, 0.0, 0.0), Pose(0.0, 0.0, 1.0, 0.0), 0.05)
    assert math.isnan(sent[-1][0])
    observed("nonfinite command reaches backend", str(sent[-1][0]))

    fake = NS(
        _now=lambda: 10.0,
        _cmd=Command("abort"),
        _pilot_override=False,
        backend=NS(state=lambda: FcState(armed=True, flying=True)),
        get_logger=lambda: logger,
    )
    fake._finish = lambda *a: FlightInterfaceNode._finish(fake, *a)
    assert FlightInterfaceNode._goal_flying(fake, None) == GoalResponse.ACCEPT
    abort = fake._cmd
    FlightInterfaceNode._start(fake, Command("hover"))
    assert abort.done.is_set() and fake._cmd.kind == "hover"
    observed("ordinary hover preempts abort", fake._cmd.kind)

    fake = NS(
        _lock=threading.RLock(),
        _now=lambda: 100.0,
        _pilot_override=False,
        p=DEFAULTS,
        _cmd=None,
        _mission_warned=False,
        _last_mission_status_s=0.0,
        _hold=Pose(0.0, 0.0, 1.0, 0.0),
    )
    fake.p = {**DEFAULTS, "mission_heartbeat_timeout_s": 5.0}
    FlightInterfaceNode._on_mission_status(fake, MissionStatus(state=MissionStatus.STATE_FAILED))
    FlightInterfaceNode._watchdogs(fake, FcState(armed=True, flying=True), fake._hold, 100.0)
    assert fake._cmd is None and fake._last_mission_status_s == 100.0
    observed("failed mission heartbeat keeps hold alive", fake._hold.z)

    # A localization publisher can disappear after its last TRACKING message.
    statuses = []
    fake = NS(
        _now=lambda: 0.0,
        cfg=SafetyConfig(),
        _loc=None,
        _loc_lost_since=None,
        _flight=NS(connected=True, armed=True, flying=True, pilot_override=False, battery_percentage=0.8),
        _flight_s=100.0,
        _link_lost_since=None,
        _corridor=None,
        _abort_requested=False,
        _last_level=0,
        get_logger=lambda: logger,
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=100))),
        status_pub=NS(publish=lambda msg: statuses.append(msg)),
    )
    SafetyMonitor._on_loc(fake, LocalizationStatus(state=LocalizationStatus.STATE_TRACKING))
    fake._now = lambda: 100.0
    SafetyMonitor._tick(fake)
    assert statuses[-1].level == 0 and not statuses[-1].abort_requested
    observed("last TRACKING status has no expiry", statuses[-1].level)

    requests = []
    fake = NS(
        _abort_requested=False,
        get_logger=lambda: logger,
        abort_client=NS(service_is_ready=lambda: True, call_async=lambda req: requests.append(req)),
    )
    SafetyMonitor._request_abort(fake, "test")
    assert fake._abort_requested and len(requests) == 1
    observed("abort latched without receiving acceptance", fake._abort_requested)

    tf = TransformStamped()
    tf.header.stamp.sec = 1
    tf.transform.rotation.w = 1.0
    fake = NS(p=DEFAULTS, _lookup=lambda *args: tf, _now=lambda: 100.0)
    assert FlightInterfaceNode._pose(fake) is not None
    observed("99-second-old TF accepted as current pose", tf.header.stamp.sec)

    state = classify(-100.0, 30.0, None, 0.5, 15.0)
    assert state == LocalizationStatus.STATE_TRACKING
    observed("future odometry classified TRACKING", state)

    with TemporaryDirectory(prefix="coredrone_audit_") as tmp:
        tmp = Path(tmp)
        # Export can overwrite files outside export_dir, using the public method.
        victim = tmp / "outside_exports" / "existing.txt"
        victim.parent.mkdir()
        victim.write_text("existing content")
        fake = NS(queue=NS(records=lambda _: []), p={"export_dir": str(tmp / "exports")})
        UplinkNode.export(fake, str(victim))
        assert victim.read_text().startswith("scan_id,")
        observed("export overwrote file outside export_dir", victim.name)

        exported = export_csv([{"code": "=1+1"}], tmp / "formula.csv")
        with exported.open(newline="") as stream:
            row = next(csv.DictReader(stream))
        assert row["code"] == "=1+1"
        observed("barcode formula survives CSV export", row["code"])

        queue = ScanQueue(tmp / "queue.sqlite")
        try:
            sample = json.loads((ROOT / "contracts/examples/scan_record.example.json").read_text())
            for i in range(3):
                record = {**sample, "scan_id": str(uuid.uuid4()), "code": "" if i < 2 else "VALID"}
                queue.put(record)
                queue._db.execute("UPDATE scans SET created_at = ? WHERE scan_id = ?", (str(i), record["scan_id"]))
                queue._db.commit()
            fake = NS(
                queue=queue,
                p={"batch_size": 2, "validate_records": True},
                backend=MockBackend(),
                synced=0,
                last_error="",
                get_logger=lambda: logger,
            )
            for _ in range(3):
                UplinkNode._sync(fake)
            assert fake.synced == 0 and queue.counts() == (3, 3)
            observed("invalid oldest batch starves valid record", queue.counts())
        finally:
            queue.close()

    data = yaml.safe_load((ROOT / "config/warehouses/sim_small.yaml").read_text())
    data = copy.deepcopy(data)
    data["rack_defaults"]["level_heights_m"] = [10.0, 0.4]
    layout = parse_layout(data)
    assert any(slot.deck_height_m == 10.0 for slot in layout.slots)
    observed("layout accepts inverted decks above rack roof", data["rack_defaults"]["level_heights_m"])
    print("All 13 audit observations reproduced.", flush=True)


if __name__ == "__main__":
    main()
