"""python3 -m worldgen --warehouse sim_small [--sensors label] [--out ../worlds/generated]

Writes <out>/<warehouse_id>/:
  world.sdf            Gazebo world (floor, lights, warehouse model, drone)
  models/              warehouse_<id>/ (racks, boxes, label textures) and inv_quad/ (drone)
  bridge.yaml          ros_gz_bridge config: canonical ROS topics for the chosen sensor set
  ground_truth.json    slot -> item and tag -> pose, for sim/evaluation
  info.json            what sim_platform.launch.py needs (spawn pose, home, files)
"""

from __future__ import annotations

import argparse
import math
import shutil
import sys
from pathlib import Path

import yaml

from worldgen import _ensure_inv_layout  # noqa: F401  (side effect: make inv_layout importable)
from worldgen.drone_model import MODEL_NAME, SPAWN_HEIGHT_M, build_drone_model
from worldgen.warehouse import assign_items, build_warehouse_model, build_world, ground_truth, write_json

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "worlds" / "generated"


def generate(warehouse: str, sensors: str, out_root: Path, with_drone: bool = True) -> Path:
    from inv_layout import load_layout

    layout = load_layout(warehouse)
    contents = assign_items(layout)
    out_dir = out_root / layout.warehouse_id
    if out_dir.exists():
        shutil.rmtree(out_dir)
    models_dir = out_dir / "models"
    models_dir.mkdir(parents=True)

    warehouse_model = build_warehouse_model(layout, contents, models_dir)
    bridge: list[dict] = []
    if with_drone:
        from inv_description.extrinsics import expand_xacro

        bridge = build_drone_model(expand_xacro(), sensors, models_dir)
    spawn = (layout.takeoff.x, layout.takeoff.y, SPAWN_HEIGHT_M, layout.takeoff.yaw)
    has_sensors = any(b["ros_type_name"].startswith("sensor_msgs") for b in bridge)
    (out_dir / "world.sdf").write_text(build_world(layout, warehouse_model, MODEL_NAME, spawn, has_sensors))
    (out_dir / "bridge.yaml").write_text(yaml.safe_dump(bridge, sort_keys=False))
    write_json(out_dir / "ground_truth.json", ground_truth(layout, contents))
    o = layout.geodetic_origin
    write_json(
        out_dir / "info.json",
        {
            "warehouse_id": layout.warehouse_id,
            "world_name": layout.warehouse_id,
            "world_file": "world.sdf",
            "sensor_set": sensors,
            "model_name": MODEL_NAME,
            "spawn": {"x": spawn[0], "y": spawn[1], "z": spawn[2], "yaw": spawn[3]},
            "geodetic_origin": {
                "latitude_deg": o.latitude_deg,
                "longitude_deg": o.longitude_deg,
                "altitude_m": o.altitude_m,
            },
            # SITL --home: lat,lon,alt(m AMSL),heading(deg). Heading 0 = facing the map's +x axis.
            "sitl_home": f"{o.latitude_deg},{o.longitude_deg},{o.altitude_m},{math.degrees(layout.takeoff.yaw):.1f}",
            "resource_paths": ["models"],
            "bridge_file": "bridge.yaml",
            "ground_truth_file": "ground_truth.json",
            "slots": len(layout.slots),
            "items": sum(1 for v in contents.values() if v is not None),
        },
    )
    return out_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--warehouse", "-w", default="sim_small", help="layout name or path")
    parser.add_argument("--sensors", default="label", help="sensor set from sim/models/inv_quad/sensors.yaml")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--no-drone", action="store_true", help="skip the drone model (no ROS needed)")
    args = parser.parse_args(argv)
    try:
        out = generate(args.warehouse, args.sensors, args.out, with_drone=not args.no_drone)
    except ModuleNotFoundError as exc:
        print(
            f"error: {exc}. Run inside the dev container after `just build` (needs inv_description, xacro).",
            file=sys.stderr,
        )
        return 1
    info = yaml.safe_load((out / "info.json").read_text())
    print(f"worldgen: {out} ({info['slots']} slots, {info['items']} items, sensors={info['sensor_set']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
