"""Warehouse world generator: layout YAML -> Gazebo world, label textures, drone model, ground truth.

Run `just worldgen W=sim_small` (or `python3 -m worldgen --help` from sim/). Output goes to
sim/worlds/generated/<warehouse>/ and is gitignored; it is fully determined by the layout file,
the sensor set and the layout's `sim.seed`, so CI runs are reproducible.
"""

import sys
from pathlib import Path


def _ensure_inv_layout() -> None:
    """Use the overlay's inv_layout if it is built, else the copy in the source tree."""
    try:
        import inv_layout  # noqa: F401
    except ImportError:
        src = Path(__file__).resolve().parents[2] / "drone" / "src" / "inv_layout"
        sys.path.insert(0, str(src))


_ensure_inv_layout()
