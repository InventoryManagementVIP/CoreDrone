"""Route generation from the warehouse layout: aisle -> rack face -> shelf level -> hover points.

Routes are generated, not hand-flown or free-planned (docs/adr/0007-py-trees.md):
  * enter each aisle from its start end, at the transit altitude
  * scan each rack face level by level; bays run serpentine (alternating direction) per level
  * leave the aisle the way it came in; move between aisles along the cross-aisle at the start end
  * return above the takeoff pad and land
Every scan point is a stop-and-hover: the label camera sits `standoff_m` from the rack face,
`camera_height_offset_m` above the shelf deck, looking at the rack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from inv_layout import Layout, Pose, body_pose_for_camera, camera_hover_pose

# Entry/exit point this far before the aisle start, on the aisle centre line.
AISLE_ENTRY_MARGIN_M = 1.0


@dataclass(frozen=True)
class Waypoint:
    name: str
    kind: str  # "transit" | "scan"
    pose: Pose  # base_link target in the map frame
    hover_s: float = 0.0
    slot_code: str = ""


def generate_route(
    layout: Layout,
    camera_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    include_racks: set[str] | None = None,
) -> list[Waypoint]:
    scan = layout.scan
    route: list[Waypoint] = []
    for aisle in layout.aisles:
        dx, dy = aisle.direction
        entry = Pose(
            aisle.start[0] - dx * AISLE_ENTRY_MARGIN_M,
            aisle.start[1] - dy * AISLE_ENTRY_MARGIN_M,
            scan.transit_altitude_m,
            aisle.heading,
        )
        route.append(Waypoint(f"{aisle.id}-entry", "transit", entry))
        for rack in aisle.racks:
            if not rack.scan or (include_racks is not None and rack.id not in include_racks):
                continue
            slots = layout.rack_slots(rack.id)
            for level in range(rack.geometry.levels):
                bays = sorted((s for s in slots if s.level == level), key=lambda s: s.bay)
                if scan.order == "serpentine" and level % 2 == 1:
                    bays.reverse()
                for slot in bays:
                    body = body_pose_for_camera(camera_hover_pose(layout, slot), camera_offset_xyz)
                    route.append(Waypoint(f"hp-{slot.code}", "scan", body, scan.hover_s, slot.code))
        route.append(Waypoint(f"{aisle.id}-exit", "transit", Pose(entry.x, entry.y, entry.z, aisle.heading + math.pi)))
    t = layout.takeoff
    route.append(Waypoint("home", "transit", Pose(t.x, t.y, scan.transit_altitude_m, t.yaw)))
    return route


def route_length_m(route: list[Waypoint], start: Pose) -> float:
    length, prev = 0.0, start
    for wp in route:
        length += math.dist((prev.x, prev.y, prev.z), (wp.pose.x, wp.pose.y, wp.pose.z))
        prev = wp.pose
    return length
