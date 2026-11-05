"""Find repo-level files (contracts, warehouse layouts) from code running inside the overlay.

The images set INV_ROOT (/ws in the dev image, /opt/inv on the Jetson). Without it we walk up from
this file, which works for `colcon build --symlink-install` and for plain-Python use from a checkout.
"""

from __future__ import annotations

import os
from pathlib import Path

ENV_VAR = "INV_ROOT"


def _looks_like_root(path: Path) -> bool:
    return (path / "contracts").is_dir() and (path / "config" / "warehouses").is_dir()


def repo_root() -> Path:
    env = os.environ.get(ENV_VAR)
    if env and _looks_like_root(Path(env)):
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if _looks_like_root(parent):
            return parent
    raise FileNotFoundError(
        f"Cannot find the repo root (a directory with contracts/ and config/warehouses/). "
        f"Set {ENV_VAR} to the repository checkout."
    )


def layout_schema_path() -> Path:
    return repo_root() / "contracts" / "warehouse_layout.schema.json"


def warehouse_path(name_or_path: str | os.PathLike[str]) -> Path:
    """Resolve a warehouse name (`sim_small`) or a path to a layout YAML file."""
    path = Path(name_or_path)
    if path.suffix in (".yaml", ".yml") and path.is_file():
        return path
    candidate = repo_root() / "config" / "warehouses" / f"{path.name}.yaml"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"No warehouse layout '{name_or_path}' (looked for {candidate})")
