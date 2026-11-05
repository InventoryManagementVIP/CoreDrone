import copy
import math

import pytest
import yaml

from inv_layout import (
    LayoutError,
    Pose,
    body_pose_for_camera,
    camera_hover_pose,
    load_layout,
    parse_layout,
    warehouse_path,
)


@pytest.fixture(scope="module")
def small():
    return load_layout("sim_small")


@pytest.fixture(scope="module")
def small_dict():
    return yaml.safe_load(warehouse_path("sim_small").read_text())


def test_all_committed_layouts_load():
    for name in ("sim_small", "sim_large"):
        layout = load_layout(name)
        assert layout.warehouse_id == name
        assert layout.slots


def test_sim_small_counts(small):
    # 2 aisles x 2 racks x 3 levels x 3 bays
    assert len(small.aisles) == 2
    assert len(small.racks()) == 4
    assert len(small.slots) == 36
    assert len({s.code for s in small.slots}) == 36


def test_tag_ids_are_sequential_and_unique(small):
    ids = [s.location_tag_id for s in small.slots]
    assert ids == list(range(36))
    assert small.slots[5].location_code == "tag36h11:5"
    assert set(small.slots_by_location_code()) == {f"tag36h11:{i}" for i in range(36)}


def test_location_labels_sit_on_rack_faces(small):
    for slot in small.slots:
        aisle = next(a for a in small.aisles if a.id == slot.aisle_id)
        # Distance of the label from the aisle centre line is half the aisle width.
        dx, dy = aisle.direction
        px, py = slot.location_label.x - aisle.start[0], slot.location_label.y - aisle.start[1]
        lateral = abs(px * -dy + py * dx)
        assert lateral == pytest.approx(aisle.width_m / 2, abs=0.01)
        # Label faces into the aisle, under the deck.
        assert slot.location_label.z < slot.deck_height_m
        assert math.cos(slot.location_label.yaw) * slot.into_aisle[0] + math.sin(
            slot.location_label.yaw
        ) * slot.into_aisle[1] == pytest.approx(1.0)


def test_right_rack_of_first_aisle(small):
    slot = small.slot("A1-R-L0-B0")
    # Aisle A1 runs along +x at y = 1.5 and is 3 m wide: the right face is y = 0.
    assert slot.face_point[1] == pytest.approx(0.0)
    assert slot.into_aisle == pytest.approx((0.0, 1.0))
    # offset 0.4 + half a 0.9 m bay from the aisle start at x = 1.0
    assert slot.face_point[0] == pytest.approx(1.85)
    assert slot.deck_height_m == pytest.approx(0.40)


def test_camera_hover_pose_looks_at_rack(small):
    slot = small.slot("A1-L-L2-B1")
    cam = camera_hover_pose(small, slot)
    # Left face of A1 is y = 3.0, standoff 1.0 m into the aisle, looking +y.
    assert cam.y == pytest.approx(2.0)
    assert cam.z == pytest.approx(2.20 + 0.10)
    assert cam.yaw == pytest.approx(math.pi / 2)


def test_body_pose_for_camera_round_trip():
    cam = Pose(1.0, 2.0, 1.5, math.radians(30))
    offset = (0.12, 0.03, -0.05)
    body = body_pose_for_camera(cam, offset)
    c, s = math.cos(body.yaw), math.sin(body.yaw)
    assert body.x + c * offset[0] - s * offset[1] == pytest.approx(cam.x)
    assert body.y + s * offset[0] + c * offset[1] == pytest.approx(cam.y)
    assert body.z + offset[2] == pytest.approx(cam.z)


def test_bounds_contain_everything(small):
    (x0, y0, _z0), (x1, y1, z1) = small.bounds(margin_m=0.0)
    for slot in small.slots:
        assert x0 <= slot.center[0] <= x1
        assert y0 <= slot.center[1] <= y1
    assert x0 <= small.takeoff.x <= x1
    assert z1 >= 3.0


def test_nearest_slot(small):
    target = small.slot("A2-L-L1-B2")
    found, dist = small.nearest_slot(*target.center)
    assert found.code == target.code
    assert dist == pytest.approx(0.0)


def test_slot_opening(small):
    # sim_small: bays 0.9 m, decks at 0.40 / 1.30 / 2.20 m, 0.12 m beams, 3.0 m uprights.
    assert small.slot_opening(small.slot("A1-R-L0-B0")) == pytest.approx((0.9, 1.30 - 0.12 - 0.40))
    assert small.slot_opening(small.slot("A2-L-L2-B1")) == pytest.approx((0.9, 3.0 - 2.20))


def test_schema_rejects_unknown_side(small_dict):
    bad = copy.deepcopy(small_dict)
    bad["aisles"][0]["racks"][0]["side"] = "up"
    with pytest.raises(LayoutError, match="side"):
        parse_layout(bad)


def test_schema_rejects_missing_scan(small_dict):
    bad = copy.deepcopy(small_dict)
    del bad["scan"]
    with pytest.raises(LayoutError, match="scan"):
        parse_layout(bad)


def test_duplicate_slot_codes_rejected(small_dict):
    bad = copy.deepcopy(small_dict)
    bad["slot_code_format"] = "{rack}-L{level}"  # drops the bay, so codes collide
    with pytest.raises(LayoutError, match="duplicate slot code"):
        parse_layout(bad)


def test_two_racks_on_one_side_rejected(small_dict):
    bad = copy.deepcopy(small_dict)
    bad["aisles"][0]["racks"][1]["side"] = "right"
    with pytest.raises(LayoutError, match="two racks"):
        parse_layout(bad)
