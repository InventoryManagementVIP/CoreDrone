"""Read ArduPilot .parm files (NAME VALUE or NAME,VALUE; # comments) and layer them."""

from __future__ import annotations

import math
import re
from pathlib import Path

_LINE = re.compile(r"^\s*([A-Z0-9_]+)\s*[ ,\t]\s*([-+0-9.eE]+)\s*(?:#.*)?$")


def parse_parm_text(text: str, source: str = "<text>") -> dict[str, float]:
    params: dict[str, float] = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        m = _LINE.match(line)
        if not m:
            raise ValueError(f"{source}:{lineno}: cannot parse {raw!r}")
        params[m.group(1)] = float(m.group(2))
    return params


def load_layers(paths: list[str | Path]) -> dict[str, tuple[float, str]]:
    """Merge files in order; later files win. Returns name -> (value, file that set it)."""
    merged: dict[str, tuple[float, str]] = {}
    for path in paths:
        path = Path(path)
        for name, value in parse_parm_text(path.read_text(), str(path)).items():
            merged[name] = (value, path.name)
    return merged


def same_value(a: float, b: float) -> bool:
    """Parameters travel as float32, so compare with float32 precision."""
    return math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-6)


def diff(expected: dict[str, tuple[float, str]], actual: dict[str, float]) -> tuple[list[tuple], list[tuple]]:
    """Return (mismatches, unknown): values that differ, and names the FC does not have."""
    mismatches, unknown = [], []
    for name, (value, source) in sorted(expected.items()):
        if name not in actual:
            unknown.append((name, value, source))
        elif not same_value(value, actual[name]):
            mismatches.append((name, value, actual[name], source))
    return mismatches, unknown
