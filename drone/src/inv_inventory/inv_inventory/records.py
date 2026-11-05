"""Scan records in the drone -> backend format (contracts/scan_record.schema.json)."""

from __future__ import annotations

import datetime as dt
import uuid

SCHEMA_VERSION = "1.0"


def rfc3339(when: dt.datetime) -> str:
    return when.astimezone(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def make_scan_record(
    *,
    mission_id: str,
    drone_id: str,
    warehouse_id: str,
    timestamp: dt.datetime,
    code: str,
    symbology: str,
    slot_code: str | None,
    rack_id: str | None,
    level: int | None,
    position_map: tuple[float, float, float] | None,
    confidence: float,
    attribution_method: str,
    image_ref: str | None = None,
    scan_id: str | None = None,
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "scan_id": scan_id or str(uuid.uuid4()),
        "mission_id": mission_id,
        "drone_id": drone_id,
        "warehouse_id": warehouse_id,
        "timestamp": rfc3339(timestamp),
        "code": code,
        "symbology": symbology,
        "slot_code": slot_code,
        "rack_id": rack_id,
        "level": level,
        "position_map": None
        if position_map is None
        else {"x": round(position_map[0], 4), "y": round(position_map[1], 4), "z": round(position_map[2], 4)},
        "confidence": max(0.0, min(1.0, float(confidence))),
        "attribution_method": attribution_method,
        "image_ref": image_ref,
    }
