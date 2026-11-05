import math
import pathlib

import pytest
import yaml

from inv_layout import camera_hover_pose, load_layout
from inv_mission.route import AISLE_ENTRY_MARGIN_M, generate_route


@pytest.fixture(scope="module")
def layout():
    return load_layout("sim_small")


def test_every_slot_is_scanned_once(layout):
    route = generate_route(layout)
    scanned = [wp.slot_code for wp in route if wp.kind == "scan"]
    assert sorted(scanned) == sorted(s.code for s in layout.slots)
    assert all(wp.hover_s == layout.scan.hover_s for wp in route if wp.kind == "scan")


def test_route_starts_at_the_first_aisle_and_ends_home(layout):
    route = generate_route(layout)
    first, last = route[0], route[-1]
    assert first.name == "A1-entry"
    assert first.pose.x == pytest.approx(layout.aisles[0].start[0] - AISLE_ENTRY_MARGIN_M)
    assert last.name == "home"
    assert (last.pose.x, last.pose.y) == pytest.approx((layout.takeoff.x, layout.takeoff.y))


def test_serpentine_bays(layout):
    route = generate_route(layout)
    a1r = [wp for wp in route if wp.slot_code.startswith("A1-R-")]
    assert [wp.slot_code for wp in a1r[:6]] == [
        "A1-R-L0-B0",
        "A1-R-L0-B1",
        "A1-R-L0-B2",
        "A1-R-L1-B2",
        "A1-R-L1-B1",
        "A1-R-L1-B0",
    ]


def test_hover_points_keep_the_camera_on_target(layout):
    offset = (0.12, -0.015, -0.03)
    route = generate_route(layout, offset)
    for wp in route:
        if wp.kind != "scan":
            continue
        cam = camera_hover_pose(layout, layout.slot(wp.slot_code))
        c, s = math.cos(wp.pose.yaw), math.sin(wp.pose.yaw)
        assert wp.pose.x + c * offset[0] - s * offset[1] == pytest.approx(cam.x)
        assert wp.pose.y + s * offset[0] + c * offset[1] == pytest.approx(cam.y)
        assert wp.pose.z + offset[2] == pytest.approx(cam.z)


def test_hover_points_stay_inside_the_aisle(layout):
    for wp in generate_route(layout):
        if wp.kind != "scan":
            continue
        aisle = next(a for a in layout.aisles if wp.slot_code.startswith(a.id))
        dx, dy = aisle.direction
        lateral = abs((wp.pose.x - aisle.start[0]) * -dy + (wp.pose.y - aisle.start[1]) * dx)
        assert lateral < aisle.width_m / 2 - 0.4  # leaves room for the airframe


def test_config_matches_defaults():
    from inv_mission.node import DEFAULTS

    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "mission.yaml"
    assert yaml.safe_load(path.read_text())["mission"]["ros__parameters"] == DEFAULTS
