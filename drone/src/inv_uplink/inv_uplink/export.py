"""CSV export of scan records (the format the process doc asks for), plus contract validation."""

from __future__ import annotations

import csv
import json
import os
from functools import lru_cache
from pathlib import Path

COLUMNS = [
    "scan_id",
    "mission_id",
    "drone_id",
    "warehouse_id",
    "timestamp",
    "code",
    "symbology",
    "slot_code",
    "rack_id",
    "level",
    "x",
    "y",
    "z",
    "confidence",
    "attribution_method",
    "image_ref",
]


def flatten(record: dict) -> dict:
    row = {k: record.get(k) for k in COLUMNS if k not in ("x", "y", "z")}
    pos = record.get("position_map") or {}
    row.update(x=pos.get("x"), y=pos.get("y"), z=pos.get("z"))
    return {k: ("" if v is None else v) for k, v in row.items()}


def export_csv(records: list[dict], path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        for record in records:
            writer.writerow(flatten(record))
    return path


def contract_path() -> Path:
    env = os.environ.get("INV_ROOT")
    candidates = [Path(env)] if env else []
    candidates += list(Path(__file__).resolve().parents)
    for root in candidates:
        p = root / "contracts" / "scan_record.schema.json"
        if p.is_file():
            return p
    raise FileNotFoundError("contracts/scan_record.schema.json not found; set INV_ROOT")


@lru_cache(maxsize=1)
def _validator():
    import jsonschema

    schema = json.loads(contract_path().read_text())
    return jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker())


def contract_errors(record: dict) -> list[str]:
    """Every way `record` violates the drone -> backend contract (empty list if valid)."""
    return [f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}" for e in _validator().iter_errors(record)]
