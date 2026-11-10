"""CSV export of scan records (the format the process doc asks for), plus contract validation."""

from __future__ import annotations

import csv
import json
import os
import re
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


# A spreadsheet runs a cell starting with one of these as a formula. Label text is untrusted: anyone
# can print a barcode. The quote makes it literal text; the queue keeps the original value.
_FORMULA_PREFIXES = ("=", "+", "-", "@", "\t", "\r")


def _spreadsheet_safe(value):
    return f"'{value}" if isinstance(value, str) and value.startswith(_FORMULA_PREFIXES) else value


def flatten(record: dict) -> dict:
    row = {k: record.get(k) for k in COLUMNS if k not in ("x", "y", "z")}
    pos = record.get("position_map") or {}
    row.update(x=pos.get("x"), y=pos.get("y"), z=pos.get("z"))
    return {k: ("" if v is None else _spreadsheet_safe(v)) for k, v in row.items()}


def safe_name_part(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", text).strip(".")


def resolve_export_path(export_dir: str | Path, filename: str) -> Path:
    """The file `filename` in `export_dir`. Raises ValueError for anything but a plain .csv name."""
    if filename != safe_name_part(filename) or not filename.endswith(".csv") or filename == ".csv":
        raise ValueError(f"export filename {filename!r} must be a plain name ending in .csv, not a path")
    return Path(export_dir) / filename


def export_csv(records: list[dict], path: str | Path) -> Path:
    """Write a new file; mode "x" also refuses an existing file or symlink at `path`."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="") as f:
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
