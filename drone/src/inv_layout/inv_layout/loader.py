"""Load and validate warehouse layout files against contracts/warehouse_layout.schema.json."""

from __future__ import annotations

import json
import os
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

from inv_layout.layout import Layout, build_layout
from inv_layout.paths import layout_schema_path, warehouse_path


class LayoutError(ValueError):
    """The layout file is missing, does not match the schema, or has inconsistent geometry."""


@lru_cache(maxsize=1)
def _validator():
    import jsonschema

    schema = json.loads(layout_schema_path().read_text())
    return jsonschema.Draft202012Validator(schema)


def validate_layout_dict(data: Any) -> None:
    """Raise LayoutError listing every schema violation in `data`."""
    errors = sorted(_validator().iter_errors(data), key=lambda e: list(e.path))
    if errors:
        lines = [f"  {'/'.join(str(p) for p in e.path) or '<root>'}: {e.message}" for e in errors]
        raise LayoutError("warehouse layout does not match the schema:\n" + "\n".join(lines))


def parse_layout(data: dict[str, Any]) -> Layout:
    validate_layout_dict(data)
    try:
        return build_layout(data)
    except (ValueError, KeyError) as exc:
        raise LayoutError(str(exc)) from exc


def load_layout(name_or_path: str | os.PathLike[str]) -> Layout:
    """Load a layout by warehouse name (`sim_small`) or file path."""
    path = warehouse_path(name_or_path) if not isinstance(name_or_path, Path) else name_or_path
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise LayoutError(f"cannot read layout {path}: {exc}") from exc
    return parse_layout(data)
