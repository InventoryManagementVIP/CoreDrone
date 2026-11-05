"""Warehouse model (racks, boxes, labels), world SDF and ground truth, all from one layout."""

from __future__ import annotations

import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

from inv_layout import Layout, Slot
from inv_layout.layout import BOX_SETBACK_M
from worldgen.labels import apriltag_label, qr_label
from worldgen.sdf import box_visual, material, model_config, pose, sub, to_string, vec

UPRIGHT_RGBA = (0.10, 0.25, 0.60, 1.0)  # blue uprights
BEAM_RGBA = (0.95, 0.45, 0.05, 1.0)  # orange beams
DECK_RGBA = (0.62, 0.62, 0.62, 1.0)
BOX_RGBA = (0.62, 0.46, 0.28, 1.0)  # cardboard
LABEL_RGBA = (1.0, 1.0, 1.0, 1.0)
UPRIGHT_SIZE_M = 0.06
BEAM_DEPTH_M = 0.05
DECK_THICKNESS_M = 0.02
DEFAULT_BOX_SIZE_M = (0.4, 0.4, 0.4)

LABEL_QUAD_OBJ = """\
# Unit quad in the y-z plane facing +x, used for every label. Texture u runs along +y (the
# viewer's right when looking at the label), v along +z.
v 0 -0.5 -0.5
v 0 0.5 -0.5
v 0 0.5 0.5
v 0 -0.5 0.5
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 1 0 0
f 1/1/1 2/2/1 3/3/1 4/4/1
"""


def assign_items(layout: Layout) -> dict[str, str | None]:
    """Seeded choice of which slots hold a box, and the item code printed on each box's label.

    Returns slot code -> decoded item code (what the drone should read), or None for an empty slot.
    """
    sim = layout.sim
    if sim is None:
        raise ValueError(f"layout {layout.warehouse_id} has no `sim` block, so it cannot be simulated")
    rng = random.Random(sim.seed)
    numbers = rng.sample(range(1, 100_000), len(layout.slots))
    payload_format = layout.item_label.payload_format or "{sku}"
    contents: dict[str, str | None] = {}
    for slot, number in zip(layout.slots, numbers, strict=True):
        if rng.random() < sim.fill_ratio:
            sku = sim.sku_format.format(n=number)
            contents[slot.code] = payload_format.format(sku=sku, slot_code=slot.code)
        else:
            contents[slot.code] = None
    return contents


def _label_visual(
    link: ET.Element,
    name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    edge_m: float,
    mesh_uri: str,
    texture_uri: str,
) -> None:
    v = sub(link, "visual", name=name)
    pose(v, x, y, z, yaw=yaw)
    mesh = sub(sub(v, "geometry"), "mesh")
    sub(mesh, "uri", mesh_uri)
    sub(mesh, "scale", vec((1.0, edge_m, edge_m)))
    material(v, LABEL_RGBA, albedo_map=texture_uri)


def _rack_point(rack, along: float, depth: float) -> tuple[float, float]:
    """Point `along` metres along the rack face from bay 0 and `depth` metres into the rack."""
    fx, fy = rack.face_xy(along)
    return fx + rack.outward[0] * depth, fy + rack.outward[1] * depth


def _rack_geometry(link: ET.Element, layout: Layout) -> None:
    for rack in layout.racks():
        g = rack.geometry
        yaw = math.atan2(rack.direction[1], rack.direction[0])
        for i in range(g.bays + 1):
            for depth, side in ((UPRIGHT_SIZE_M / 2, "front"), (g.depth_m - UPRIGHT_SIZE_M / 2, "back")):
                x, y = _rack_point(rack, i * g.bay_width_m, depth)
                box_visual(
                    link,
                    f"{rack.id}_upright_{side}_{i}",
                    (UPRIGHT_SIZE_M, UPRIGHT_SIZE_M, g.height_m),
                    (x, y, g.height_m / 2, yaw),
                    UPRIGHT_RGBA,
                )
        for level, deck in enumerate(g.level_heights_m):
            for depth, side in ((BEAM_DEPTH_M / 2, "front"), (g.depth_m - BEAM_DEPTH_M / 2, "back")):
                x, y = _rack_point(rack, g.length_m / 2, depth)
                box_visual(
                    link,
                    f"{rack.id}_beam_{side}_L{level}",
                    (g.length_m, BEAM_DEPTH_M, g.beam_height_m),
                    (x, y, deck - g.beam_height_m / 2, yaw),
                    BEAM_RGBA,
                )
            x, y = _rack_point(rack, g.length_m / 2, g.depth_m / 2)
            box_visual(
                link,
                f"{rack.id}_deck_L{level}",
                (g.length_m, g.depth_m, DECK_THICKNESS_M),
                (x, y, deck - DECK_THICKNESS_M / 2, yaw),
                DECK_RGBA,
            )


def _boxes(link: ET.Element, layout: Layout, contents: dict[str, str | None]) -> None:
    bx, by, bz = layout.sim.box_size_m if layout.sim else DEFAULT_BOX_SIZE_M
    for slot in layout.slots:
        if contents.get(slot.code) is None:
            continue
        ox, oy = -slot.into_aisle[0], -slot.into_aisle[1]
        x = slot.face_point[0] + ox * (BOX_SETBACK_M + by / 2)
        y = slot.face_point[1] + oy * (BOX_SETBACK_M + by / 2)
        yaw = math.atan2(slot.into_aisle[1], slot.into_aisle[0]) + math.pi / 2  # box x along the rack
        box_visual(link, f"box_{slot.code}", (bx, by, bz), (x, y, slot.deck_height_m + bz / 2, yaw), BOX_RGBA)


def build_warehouse_model(layout: Layout, contents: dict[str, str | None], models_dir: Path) -> str:
    """Write models/<name>/ (model.sdf, model.config, meshes/, textures/). Returns the model name."""
    name = f"warehouse_{layout.warehouse_id}"
    root_dir = models_dir / name
    (root_dir / "meshes").mkdir(parents=True, exist_ok=True)
    (root_dir / "textures").mkdir(parents=True, exist_ok=True)
    (root_dir / "meshes" / "label_quad.obj").write_text(LABEL_QUAD_OBJ)
    mesh_uri = f"model://{name}/meshes/label_quad.obj"

    sdf = ET.Element("sdf", version="1.9")
    model = sub(sdf, "model", name=name)
    sub(model, "static", "true")
    link = sub(model, "link", name="structure")
    _rack_geometry(link, layout)
    _boxes(link, layout, contents)

    loc, item = layout.location_label, layout.item_label
    for slot in layout.slots:
        if loc.kind == "apriltag":
            label = apriltag_label(slot.location_tag_id)
            texture = f"loc_tag36h11_{slot.location_tag_id:03d}.png"
        else:
            label = qr_label(slot.location_code)
            texture = f"loc_{slot.code}.png"
        label.image.save(root_dir / "textures" / texture)
        lp = slot.location_label
        _label_visual(
            link,
            f"label_loc_{slot.code}",
            lp.x,
            lp.y,
            lp.z,
            lp.yaw,
            loc.size_m * label.label_scale,
            mesh_uri,
            f"model://{name}/textures/{texture}",
        )

        payload = contents.get(slot.code)
        if payload is None:
            continue
        if item.kind != "qr":
            # TODO(phase-2): 1D barcode (code128 / EAN-13) item labels.
            raise ValueError("sim item labels support kind: qr only for now")
        label = qr_label(payload)
        texture = f"item_{slot.code}.png"
        label.image.save(root_dir / "textures" / texture)
        ip = slot.item_label
        _label_visual(
            link,
            f"label_item_{slot.code}",
            ip.x,
            ip.y,
            ip.z,
            ip.yaw,
            item.size_m * label.label_scale,
            mesh_uri,
            f"model://{name}/textures/{texture}",
        )

    (root_dir / "model.sdf").write_text(to_string(sdf))
    (root_dir / "model.config").write_text(
        model_config(name, f"Generated from config/warehouses/{layout.warehouse_id}.yaml")
    )
    return name


def build_world(
    layout: Layout, warehouse_model: str, drone_model: str, spawn: tuple[float, float, float, float], with_sensors: bool
) -> str:
    sdf = ET.Element("sdf", version="1.9")
    world = sub(sdf, "world", name=layout.warehouse_id)
    physics = sub(world, "physics", name="1ms", type="ignore")
    sub(physics, "max_step_size", "0.001")
    sub(physics, "real_time_factor", "1.0")

    plugins = [
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ]
    for filename, name in plugins:
        sub(world, "plugin", filename=filename, name=name)
    if with_sensors:
        sensors = sub(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
        sub(sensors, "render_engine", "ogre2")

    scene = sub(world, "scene")
    sub(scene, "ambient", "0.75 0.75 0.75 1")
    sub(scene, "background", "0.8 0.8 0.85 1")
    sub(scene, "shadows", "false")
    sub(scene, "grid", "false")
    # Warehouse-style lighting: one overhead light plus two that light the rack faces.
    for name, direction in (
        ("overhead", (0.1, 0.1, -1.0)),
        ("fill_north", (0.2, 1.0, -0.4)),
        ("fill_south", (-0.2, -1.0, -0.4)),
    ):
        light = sub(world, "light", type="directional", name=name)
        sub(light, "cast_shadows", "false")
        pose(light, 0, 0, 10)
        sub(light, "diffuse", "0.7 0.7 0.7 1")
        sub(light, "specular", "0.1 0.1 0.1 1")
        sub(light, "direction", vec(direction))

    geo = layout.geodetic_origin
    sph = sub(world, "spherical_coordinates")
    sub(sph, "surface_model", "EARTH_WGS84")
    sub(sph, "world_frame_orientation", "ENU")
    sub(sph, "latitude_deg", f"{geo.latitude_deg:.8f}")
    sub(sph, "longitude_deg", f"{geo.longitude_deg:.8f}")
    sub(sph, "elevation", f"{geo.altitude_m:.3f}")
    sub(sph, "heading_deg", "0")

    ground = sub(world, "model", name="ground_plane")
    sub(ground, "static", "true")
    glink = sub(ground, "link", name="link")
    for tag in ("collision", "visual"):
        el = sub(glink, tag, name=tag)
        plane = sub(sub(el, "geometry"), "plane")
        sub(plane, "normal", "0 0 1")
        sub(plane, "size", "200 200")
        if tag == "visual":
            material(el, (0.55, 0.55, 0.52, 1.0))

    for uri, name, p in ((warehouse_model, "warehouse", (0, 0, 0, 0)), (drone_model, drone_model, spawn)):
        inc = sub(world, "include")
        sub(inc, "uri", f"model://{uri}")
        sub(inc, "name", name)
        pose(inc, p[0], p[1], p[2], yaw=p[3])
    return to_string(sdf)


def ground_truth(layout: Layout, contents: dict[str, str | None]) -> dict:
    slots = []
    for slot in layout.slots:
        slots.append(
            {
                "slot_code": slot.code,
                "aisle_id": slot.aisle_id,
                "rack_id": slot.rack_id,
                "level": slot.level,
                "bay": slot.bay,
                "location_code": slot.location_code,
                "location_tag_id": slot.location_tag_id,
                "location_label_position": [round(v, 4) for v in slot.location_label.as_tuple()[:3]],
                "item_code": contents.get(slot.code),
                "item_label_position": [round(v, 4) for v in slot.item_label.as_tuple()[:3]],
            }
        )
    return {
        "schema_version": "1.0",
        "warehouse_id": layout.warehouse_id,
        "seed": layout.sim.seed if layout.sim else None,
        "slots": slots,
        # item code (as decoded) -> slot code
        "items": {item: code for code, item in contents.items() if item is not None},
    }


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=False) + "\n")


__all__ = ["Slot", "assign_items", "build_warehouse_model", "build_world", "ground_truth", "write_json"]
