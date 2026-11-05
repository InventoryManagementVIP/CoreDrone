"""`ros2 run inv_mission route <warehouse>`: print the generated route without flying it."""

from __future__ import annotations

import argparse
import math
import sys

from inv_layout import Pose, load_layout
from inv_mission.route import generate_route, route_length_m


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("warehouse")
    parser.add_argument(
        "--camera-offset",
        type=float,
        nargs=3,
        default=(0.12, -0.015, -0.03),
        metavar=("X", "Y", "Z"),
        help="label camera position in base_link",
    )
    args = parser.parse_args(argv)
    layout = load_layout(args.warehouse)
    route = generate_route(layout, tuple(args.camera_offset))
    t = layout.takeoff
    for i, wp in enumerate(route):
        p = wp.pose
        print(
            f"{i:3d} {wp.kind:7s} {wp.name:22s} x={p.x:6.2f} y={p.y:6.2f} z={p.z:5.2f} yaw={math.degrees(p.yaw):6.1f}"
        )
    scans = sum(1 for wp in route if wp.kind == "scan")
    print(f"{len(route)} waypoints, {scans} hover points, {route_length_m(route, Pose(t.x, t.y, 0.0, t.yaw)):.1f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
