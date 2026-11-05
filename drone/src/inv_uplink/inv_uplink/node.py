"""Uplink node: SQLite queue -> backend, whenever the network allows. Never on the flight path.

  service  /uplink/export_csv (inv_interfaces/srv/ExportCsv)
  topic    /diagnostics       (queue size, synced count, last error)
On shutdown it exports a CSV of everything queued, so a run always leaves a file behind.
"""

from __future__ import annotations

import datetime as dt
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from inv_interfaces.srv import ExportCsv
from inv_inventory.queue import ScanQueue
from inv_uplink.backends import BackendError, make_backend
from inv_uplink.export import contract_errors, export_csv

DEFAULTS: dict[str, object] = {
    "db_path": "/tmp/coredrone/inventory.sqlite",
    "backend": "mock",
    "mock_output": "/tmp/coredrone/mock_backend.jsonl",
    "mock_failure_rate": 0.0,
    "sync_period_s": 2.0,
    "batch_size": 50,
    "export_dir": "/tmp/coredrone/exports",
    "export_on_shutdown": True,
    "validate_records": True,
}


class UplinkNode(Node):
    def __init__(self) -> None:
        super().__init__("uplink")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.queue = ScanQueue(self.p["db_path"])
        kwargs = {}
        if self.p["backend"] == "mock":
            kwargs = {"output": self.p["mock_output"], "failure_rate": self.p["mock_failure_rate"]}
        self.backend = make_backend(self.p["backend"], **kwargs)
        self.synced = 0
        self.last_error = ""
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_service(ExportCsv, "/uplink/export_csv", self._on_export)
        self.create_timer(self.p["sync_period_s"], self._sync)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(f"uplink: {self.p['db_path']} -> {self.backend.name} backend")

    def _sync(self) -> None:
        batch = self.queue.pending(self.p["batch_size"])
        if not batch:
            return
        if self.p["validate_records"]:
            bad = {r["scan_id"]: errs for r in batch if (errs := contract_errors(r))}
            if bad:
                for scan_id, errs in bad.items():
                    self.get_logger().error(f"record {scan_id} violates the contract: {errs}")
                self.queue.mark_failed(list(bad), "contract violation")
                batch = [r for r in batch if r["scan_id"] not in bad]
        try:
            accepted = self.backend.upsert(batch)
        except BackendError as exc:
            self.last_error = str(exc)
            self.queue.mark_failed([r["scan_id"] for r in batch], self.last_error)
            self.get_logger().warning(f"sync failed, will retry: {exc}", throttle_duration_sec=10.0)
            return
        self.queue.mark_synced(accepted)
        self.synced += len(accepted)
        self.last_error = ""

    def export(self, path: str = "", mission_id: str = "") -> tuple[Path, int]:
        records = self.queue.records(mission_id or None)
        if not path:
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = str(Path(self.p["export_dir"]) / f"scans_{mission_id or 'all'}_{stamp}.csv")
        return export_csv(records, path), len(records)

    def _on_export(self, request: ExportCsv.Request, response: ExportCsv.Response) -> ExportCsv.Response:
        try:
            path, count = self.export(request.path, request.mission_id)
            response.success, response.path, response.count = True, str(path), count
            response.message = f"exported {count} records"
        except OSError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _publish_diagnostics(self) -> None:
        total, pending = self.queue.counts()
        status = DiagnosticStatus(name="uplink", hardware_id=self.backend.name)
        status.level = DiagnosticStatus.WARN if self.last_error else DiagnosticStatus.OK
        status.message = self.last_error or f"{pending} pending"
        status.values = [
            KeyValue(key="queued_total", value=str(total)),
            KeyValue(key="pending", value=str(pending)),
            KeyValue(key="synced_this_run", value=str(self.synced)),
        ]
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(msg)

    def shutdown(self) -> None:
        if self.p["export_on_shutdown"]:
            path, count = self.export()
            self.get_logger().info(f"exported {count} records to {path}")
        self.queue.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UplinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
