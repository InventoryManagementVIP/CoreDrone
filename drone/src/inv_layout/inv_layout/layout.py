"""Warehouse layout model and the geometry derived from it.

Conventions (docs/interfaces.md):
  * Map frame: ENU, metres, z = 0 on the floor.
  * A yaw is the direction an object's +x axis points. A label's yaw is the direction its printed
    face looks (into the aisle). A camera's yaw is the direction it looks.
  * Racks stand beside an aisle centre line. `side` is left/right looking from the aisle start to
    its end. The rack face is width_m / 2 from the centre line; the rack extends away from the aisle.
  * Slot (level, bay): level 0 is the lowest shelf deck, bay 0 is nearest the aisle start.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

# Labels are modelled as sitting this far in front of the surface they are stuck on.
LABEL_SURFACE_OFFSET_M = 0.002
# Boxes sit this far behind the rack face.
BOX_SETBACK_M = 0.05
# Item label height above the deck when the layout has no `sim.box_size_m`.
DEFAULT_ITEM_LABEL_HEIGHT_M = 0.2


def _unit(dx: float, dy: float) -> Vec2:
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("zero-length direction")
    return (dx / norm, dy / norm)


def _add(a: Vec2, b: Vec2, scale: float = 1.0) -> Vec2:
    return (a[0] + b[0] * scale, a[1] + b[1] * scale)


def _yaw(v: Vec2) -> float:
    return math.atan2(v[1], v[0])


@dataclass(frozen=True)
class Pose:
    """Position in the map frame plus yaw (radians)."""

    x: float
    y: float
    z: float
    yaw: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.yaw)


@dataclass(frozen=True)
class GeodeticOrigin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class RackGeometry:
    bays: int
    bay_width_m: float
    depth_m: float
    level_heights_m: tuple[float, ...]
    height_m: float
    beam_height_m: float

    @property
    def levels(self) -> int:
        return len(self.level_heights_m)

    @property
    def length_m(self) -> float:
        return self.bays * self.bay_width_m


@dataclass(frozen=True)
class LabelSpec:
    kind: str
    size_m: float
    family: str | None = None
    first_id: int = 0
    payload_format: str | None = None


@dataclass(frozen=True)
class ScanSettings:
    standoff_m: float
    hover_s: float
    camera_height_offset_m: float = 0.1
    transit_altitude_m: float = 1.0
    speed_mps: float = 0.5
    order: str = "serpentine"


@dataclass(frozen=True)
class SimSettings:
    seed: int = 0
    fill_ratio: float = 1.0
    sku_format: str = "SKU-{n:05d}"
    box_size_m: Vec3 = (0.4, 0.4, 0.4)


@dataclass(frozen=True)
class Slot:
    code: str
    aisle_id: str
    rack_id: str
    side: str
    level: int
    bay: int
    deck_height_m: float
    # Middle of the slot opening on the rack face, at deck height.
    face_point: Vec3
    # Middle of the slot volume, at item-label height.
    center: Vec3
    # Horizontal unit vector from the rack face into the aisle.
    into_aisle: Vec2
    # Location label (or tag) on the beam under the slot, facing the aisle.
    location_label: Pose
    # Where an item's label sits: front face of a box on this slot, facing the aisle.
    item_label: Pose
    # AprilTag ID of the location label, when location labels are AprilTags.
    location_tag_id: int | None
    # Payload a detector reports for the location label: "tag36h11:5" or the printed text.
    location_code: str


@dataclass(frozen=True)
class Rack:
    id: str
    aisle_id: str
    side: str
    offset_m: float
    geometry: RackGeometry
    scan: bool
    # Point where the rack face begins (edge of bay 0), and unit vectors along the face and
    # from the face into the rack.
    face_start: Vec2
    direction: Vec2
    outward: Vec2

    def face_xy(self, along_m: float) -> Vec2:
        """Point on the rack face `along_m` metres from the bay-0 edge."""
        return _add(self.face_start, self.direction, along_m)

    def footprint(self) -> list[Vec2]:
        """Rack footprint corners on the floor, counter-clockwise or clockwise."""
        g = self.geometry
        a = self.face_start
        b = self.face_xy(g.length_m)
        return [a, b, _add(b, self.outward, g.depth_m), _add(a, self.outward, g.depth_m)]


@dataclass(frozen=True)
class Aisle:
    id: str
    start: Vec2
    end: Vec2
    width_m: float
    racks: tuple[Rack, ...]

    @property
    def direction(self) -> Vec2:
        return _unit(self.end[0] - self.start[0], self.end[1] - self.start[1])

    @property
    def heading(self) -> float:
        return _yaw(self.direction)

    @property
    def length_m(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])


@dataclass
class Layout:
    warehouse_id: str
    name: str
    geodetic_origin: GeodeticOrigin
    takeoff: Pose
    location_label: LabelSpec
    item_label: LabelSpec
    slot_code_format: str
    scan: ScanSettings
    aisles: tuple[Aisle, ...]
    sim: SimSettings | None
    slots: tuple[Slot, ...]
    raw: dict[str, Any] = field(repr=False, default_factory=dict)

    # ---- lookups ---------------------------------------------------------------------------
    def slot(self, code: str) -> Slot:
        for slot in self.slots:
            if slot.code == code:
                return slot
        raise KeyError(code)

    def slots_by_location_code(self) -> dict[str, Slot]:
        return {slot.location_code: slot for slot in self.slots}

    def racks(self) -> list[Rack]:
        return [rack for aisle in self.aisles for rack in aisle.racks]

    def rack(self, rack_id: str) -> Rack:
        for rack in self.racks():
            if rack.id == rack_id:
                return rack
        raise KeyError(rack_id)

    def rack_slots(self, rack_id: str) -> list[Slot]:
        return [slot for slot in self.slots if slot.rack_id == rack_id]

    def slot_opening(self, slot: Slot) -> tuple[float, float]:
        """Clear opening of a slot on the rack face: (width, height) in metres. The height runs
        from the slot's deck up to the underside of the beam above it, or to the rack top."""
        g = self.rack(slot.rack_id).geometry
        if slot.level + 1 < g.levels:
            top = g.level_heights_m[slot.level + 1] - g.beam_height_m
        else:
            top = g.height_m
        return g.bay_width_m, top - slot.deck_height_m

    def nearest_slot(self, x: float, y: float, z: float) -> tuple[Slot, float]:
        """Slot whose centre is nearest to a map-frame point, and the distance to it."""
        best = min(self.slots, key=lambda s: math.dist(s.center, (x, y, z)))
        return best, math.dist(best.center, (x, y, z))

    def bounds(self, margin_m: float = 1.0) -> tuple[Vec3, Vec3]:
        """Axis-aligned box around all racks, aisles and the takeoff pad, plus a margin."""
        xs = [self.takeoff.x]
        ys = [self.takeoff.y]
        zmax = self.scan.transit_altitude_m
        for aisle in self.aisles:
            xs += [aisle.start[0], aisle.end[0]]
            ys += [aisle.start[1], aisle.end[1]]
            for rack in aisle.racks:
                for cx, cy in rack.footprint():
                    xs.append(cx)
                    ys.append(cy)
                zmax = max(zmax, rack.geometry.height_m)
        return (
            (min(xs) - margin_m, min(ys) - margin_m, 0.0),
            (max(xs) + margin_m, max(ys) + margin_m, zmax + margin_m),
        )


def camera_hover_pose(layout: Layout, slot: Slot) -> Pose:
    """Where the label camera should be to scan `slot`: on the aisle side, looking at the rack."""
    into = slot.into_aisle
    x = slot.face_point[0] + into[0] * layout.scan.standoff_m
    y = slot.face_point[1] + into[1] * layout.scan.standoff_m
    z = slot.deck_height_m + layout.scan.camera_height_offset_m
    return Pose(x, y, z, _yaw((-into[0], -into[1])))


def body_pose_for_camera(camera: Pose, camera_offset_xyz: Vec3) -> Pose:
    """base_link pose that puts a forward-looking camera at `camera`.

    `camera_offset_xyz` is the camera position in base_link (from the URDF via TF). The camera is
    assumed to look along base_link +x, so the body yaw equals the camera yaw.
    """
    cx, cy, cz = camera_offset_xyz
    c, s = math.cos(camera.yaw), math.sin(camera.yaw)
    return Pose(camera.x - (c * cx - s * cy), camera.y - (s * cx + c * cy), camera.z - cz, camera.yaw)


# ---- construction ------------------------------------------------------------------------------


def _geometry(defaults: dict[str, Any], override: dict[str, Any] | None) -> RackGeometry:
    merged = dict(defaults)
    merged.update(override or {})
    return RackGeometry(
        bays=int(merged["bays"]),
        bay_width_m=float(merged["bay_width_m"]),
        depth_m=float(merged["depth_m"]),
        level_heights_m=tuple(float(h) for h in merged["level_heights_m"]),
        height_m=float(merged["height_m"]),
        beam_height_m=float(merged["beam_height_m"]),
    )


def _label_spec(data: dict[str, Any]) -> LabelSpec:
    return LabelSpec(
        kind=data["kind"],
        size_m=float(data["size_m"]),
        family=data.get("family"),
        first_id=int(data.get("first_id", 0)),
        payload_format=data.get("payload_format"),
    )


def build_layout(data: dict[str, Any]) -> Layout:
    """Build a Layout from an already schema-validated dict. Raises ValueError on bad geometry."""
    origin = data["geodetic_origin"]
    takeoff = data["takeoff"]
    scan = data["scan"]
    location_label = _label_spec(data["labels"]["location"])
    item_label = _label_spec(data["labels"]["item"])
    sim = None
    if "sim" in data:
        sd = data["sim"]
        sim = SimSettings(
            seed=int(sd.get("seed", 0)),
            fill_ratio=float(sd.get("fill_ratio", 1.0)),
            sku_format=sd.get("sku_format", "SKU-{n:05d}"),
            box_size_m=tuple(float(v) for v in sd.get("box_size_m", (0.4, 0.4, 0.4))),
        )
    item_label_height = sim.box_size_m[2] / 2 if sim else DEFAULT_ITEM_LABEL_HEIGHT_M

    aisles: list[Aisle] = []
    slots: list[Slot] = []
    seen_codes: set[str] = set()
    for aisle_data in data["aisles"]:
        start = (float(aisle_data["start"]["x"]), float(aisle_data["start"]["y"]))
        end = (float(aisle_data["end"]["x"]), float(aisle_data["end"]["y"]))
        width = float(aisle_data["width_m"])
        direction = _unit(end[0] - start[0], end[1] - start[1])
        left = (-direction[1], direction[0])
        racks: list[Rack] = []
        sides_used: set[str] = set()
        for rack_data in aisle_data["racks"]:
            side = rack_data["side"]
            if side in sides_used:
                raise ValueError(f"aisle {aisle_data['id']}: two racks on the {side} side")
            sides_used.add(side)
            outward = left if side == "left" else (-left[0], -left[1])
            offset = float(rack_data.get("offset_m", 0.0))
            geometry = _geometry(data["rack_defaults"], rack_data.get("geometry"))
            face_start = _add(_add(start, direction, offset), outward, width / 2)
            rack = Rack(
                id=rack_data["id"],
                aisle_id=aisle_data["id"],
                side=side,
                offset_m=offset,
                geometry=geometry,
                scan=bool(rack_data.get("scan", True)),
                face_start=face_start,
                direction=direction,
                outward=outward,
            )
            racks.append(rack)
            into = (-outward[0], -outward[1])
            label_yaw = _yaw(into)
            for level, deck in enumerate(geometry.level_heights_m):
                for bay in range(geometry.bays):
                    fx, fy = rack.face_xy((bay + 0.5) * geometry.bay_width_m)
                    code = data["slot_code_format"].format(
                        aisle=aisle_data["id"], rack=rack.id, side=side, level=level, bay=bay
                    )
                    if code in seen_codes:
                        raise ValueError(f"duplicate slot code {code!r}: fix slot_code_format")
                    seen_codes.add(code)
                    tag_id = None
                    if location_label.kind == "apriltag":
                        tag_id = location_label.first_id + len(slots)
                        location_code = f"{location_label.family}:{tag_id}"
                    else:
                        fmt = location_label.payload_format or "{slot_code}"
                        location_code = fmt.format(slot_code=code, sku="")
                    loc_xy = _add((fx, fy), into, LABEL_SURFACE_OFFSET_M)
                    item_xy = _add((fx, fy), outward, BOX_SETBACK_M - LABEL_SURFACE_OFFSET_M)
                    center_xy = _add((fx, fy), outward, geometry.depth_m / 2)
                    slots.append(
                        Slot(
                            code=code,
                            aisle_id=aisle_data["id"],
                            rack_id=rack.id,
                            side=side,
                            level=level,
                            bay=bay,
                            deck_height_m=deck,
                            face_point=(fx, fy, deck),
                            center=(center_xy[0], center_xy[1], deck + item_label_height),
                            into_aisle=into,
                            location_label=Pose(loc_xy[0], loc_xy[1], deck - geometry.beam_height_m / 2, label_yaw),
                            item_label=Pose(item_xy[0], item_xy[1], deck + item_label_height, label_yaw),
                            location_tag_id=tag_id,
                            location_code=location_code,
                        )
                    )
        aisles.append(Aisle(aisle_data["id"], start, end, width, tuple(racks)))

    if location_label.kind == "apriltag" and slots and slots[-1].location_tag_id > 586:
        raise ValueError("tag36h11 has IDs 0..586; this layout needs more location tags")

    return Layout(
        warehouse_id=data["warehouse_id"],
        name=data.get("name", data["warehouse_id"]),
        geodetic_origin=GeodeticOrigin(
            float(origin["latitude_deg"]), float(origin["longitude_deg"]), float(origin["altitude_m"])
        ),
        takeoff=Pose(float(takeoff["x"]), float(takeoff["y"]), 0.0, math.radians(float(takeoff.get("yaw_deg", 0.0)))),
        location_label=location_label,
        item_label=item_label,
        slot_code_format=data["slot_code_format"],
        scan=ScanSettings(
            standoff_m=float(scan["standoff_m"]),
            hover_s=float(scan["hover_s"]),
            camera_height_offset_m=float(scan.get("camera_height_offset_m", 0.1)),
            transit_altitude_m=float(scan.get("transit_altitude_m", 1.0)),
            speed_mps=float(scan.get("speed_mps", 0.5)),
            order=scan.get("order", "serpentine"),
        ),
        aisles=tuple(aisles),
        sim=sim,
        slots=tuple(slots),
        raw=data,
    )
