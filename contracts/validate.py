#!/usr/bin/env python3
"""Validate the contracts themselves (CI `contracts` job, `just test-sim`).

  * both JSON Schemas are valid Draft 2020-12 schemas
  * contracts/examples/*.json validate against their schema
  * every config/warehouses/*.yaml validates against warehouse_layout.schema.json and has
    consistent geometry (via inv_layout, pure Python)
Needs only jsonschema and PyYAML, so it also runs outside the dev container.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import jsonschema
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "drone" / "src" / "inv_layout"))


def main() -> int:
    errors: list[str] = []
    schemas = {}
    for name in ("scan_record", "warehouse_layout"):
        path = ROOT / "contracts" / f"{name}.schema.json"
        schema = json.loads(path.read_text())
        try:
            jsonschema.Draft202012Validator.check_schema(schema)
        except jsonschema.SchemaError as exc:
            errors.append(f"{path}: invalid schema: {exc.message}")
        schemas[name] = jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker())

    for example in sorted((ROOT / "contracts" / "examples").glob("scan_record*.json")):
        errors += [
            f"{example}: {e.message}" for e in schemas["scan_record"].iter_errors(json.loads(example.read_text()))
        ]

    from inv_layout import LayoutError, parse_layout

    layouts = sorted((ROOT / "config" / "warehouses").glob("*.yaml"))
    for path in layouts:
        try:
            parse_layout(yaml.safe_load(path.read_text()))
        except LayoutError as exc:
            errors.append(f"{path}: {exc}")

    for e in errors:
        print(f"FAIL {e}")
    print(f"contracts: 2 schemas, {len(layouts)} warehouse layouts: {'FAIL' if errors else 'OK'}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
