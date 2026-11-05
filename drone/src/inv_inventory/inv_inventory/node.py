"""Inventory node: detections + TF + warehouse layout -> attributed items.

  in   /perception/detections (LabelDetectionArray, several detectors per image)
       /mission/status        (mission id)
       TF map <- camera optical frame (to put labels in the map)
  out  /inventory/items       (InventoryItemArray)
       SQLite queue at db_path (scan records for inv_uplink; contracts/scan_record.schema.json)

Detections that share an image stamp are processed together once `group_delay_s` has passed,
so the barcode reader and the AprilTag detector can both contribute to one frame.
"""

from __future__ import annotations

import datetime as dt
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from inv_interfaces.msg import InventoryItem, InventoryItemArray, LabelDetectionArray, MissionStatus
from inv_inventory.attribution import Attributor, Detection, better
from inv_inventory.queue import ScanQueue
from inv_inventory.records import make_scan_record

DEFAULTS: dict[str, object] = {
    "warehouse": "sim_small",
    "mission_id": "",
    "drone_id": "coredrone-1",
    "db_path": "/tmp/coredrone/inventory.sqlite",
    "map_frame": "map",
    "group_delay_s": 0.3,
    "label_window_s": 1.0,
    "max_pose_distance_m": 0.6,
    "slot_margin_m": 0.1,
}


def _quat_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class InventoryNode(Node):
    def __init__(self) -> None:
        super().__init__("inventory")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}

        from inv_layout import load_layout

        self.layout = load_layout(self.p["warehouse"])
        self.attributor = Attributor(
            self.layout, self.p["label_window_s"], self.p["max_pose_distance_m"], self.p["slot_margin_m"]
        )
        self.queue = ScanQueue(self.p["db_path"])
        self.mission_id = self.p["mission_id"]
        self._best: dict[str, str] = {}  # code -> best attribution method recorded this mission
        self._pending: dict[tuple[int, int], dict] = {}  # stamp -> {"msgs": [...], "first": wall time}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # see main(): multi-threaded executor
        self.items_pub = self.create_publisher(InventoryItemArray, "/inventory/items", 10)
        self.create_subscription(LabelDetectionArray, "/perception/detections", self._on_detections, 50)
        self.create_subscription(MissionStatus, "/mission/status", self._on_mission, 10)
        self.create_timer(0.1, self._flush)
        self.get_logger().info(
            f"inventory: {self.layout.warehouse_id} ({len(self.layout.slots)} slots) -> {self.p['db_path']}"
        )

    def _on_mission(self, msg: MissionStatus) -> None:
        if msg.mission_id and msg.mission_id != self.mission_id and not self.p["mission_id"]:
            self.mission_id = msg.mission_id
            self._best.clear()
            self.get_logger().info(f"mission {self.mission_id}")

    def _on_detections(self, msg: LabelDetectionArray) -> None:
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        group = self._pending.setdefault(key, {"msgs": [], "first": time.monotonic()})
        group["msgs"].append(msg)

    def _flush(self) -> None:
        now = time.monotonic()
        for key in sorted(k for k, g in self._pending.items() if now - g["first"] >= self.p["group_delay_s"]):
            self._process(self._pending.pop(key)["msgs"])

    def _to_map(self, msg: LabelDetectionArray, det) -> tuple[float, float, float] | None:
        if not det.has_pose:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.p["map_frame"], msg.header.frame_id, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.1)
            )
        except TransformException:
            return None
        r = _quat_matrix(t.transform.rotation)
        tr = t.transform.translation
        p = det.pose.position
        x, y, z = r @ np.array([p.x, p.y, p.z]) + np.array([tr.x, tr.y, tr.z])
        return (float(x), float(y), float(z))

    def _process(self, msgs: list[LabelDetectionArray]) -> None:
        header = msgs[0].header
        stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9
        detections, raw = [], {}
        for msg in msgs:
            for d in msg.detections:
                det = Detection(d.code, d.symbology, (d.center_x_px, d.center_y_px), d.confidence, self._to_map(msg, d))
                detections.append(det)
                raw[d.code] = det
        if not detections:
            return
        out = InventoryItemArray()
        out.header.stamp = header.stamp
        out.header.frame_id = self.p["map_frame"]
        # Sim time starts at 0, which is not a real date; use the wall clock for the record then.
        when = (
            dt.datetime.fromtimestamp(stamp_s, dt.timezone.utc) if stamp_s > 1e9 else dt.datetime.now(dt.timezone.utc)
        )
        for a in self.attributor.attribute_frame(stamp_s, detections):
            if not better(a.method, self._best.get(a.item.code)):
                continue
            self._best[a.item.code] = a.method
            record = make_scan_record(
                mission_id=self.mission_id or "unknown",
                drone_id=self.p["drone_id"],
                warehouse_id=self.layout.warehouse_id,
                timestamp=when,
                code=a.item.code,
                symbology=a.item.symbology,
                slot_code=a.slot.code if a.slot else None,
                rack_id=a.slot.rack_id if a.slot else None,
                level=a.slot.level if a.slot else None,
                position_map=a.item.position_map,
                confidence=a.item.confidence,
                attribution_method=a.method,
            )
            self.queue.put(record)
            item = InventoryItem()
            item.header = out.header
            item.scan_id, item.mission_id, item.code = record["scan_id"], record["mission_id"], a.item.code
            item.symbology = a.item.symbology
            item.slot_code = record["slot_code"] or ""
            item.rack_id = record["rack_id"] or ""
            item.level = record["level"] if record["level"] is not None else -1
            item.has_position = a.item.position_map is not None
            if item.has_position:
                item.position_map.x, item.position_map.y, item.position_map.z = a.item.position_map
            item.confidence = float(record["confidence"])
            item.attribution_method = a.method
            out.items.append(item)
            self.get_logger().info(f"item {a.item.code} -> {item.slot_code or '?'} ({a.method})")
        if out.items:
            self.items_pub.publish(out)

    def destroy_node(self) -> bool:
        self.queue.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InventoryNode()
    # Multi-threaded so TF keeps arriving (on the listener's reentrant callback group) while a
    # callback waits for a transform. The node's own callbacks share one mutually exclusive group.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
