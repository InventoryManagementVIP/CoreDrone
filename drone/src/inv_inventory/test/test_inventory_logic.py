import datetime as dt
import json
import pathlib
import threading

import pytest
import yaml

from inv_inventory.attribution import LABEL, NONE, POSE, Attributor, Detection, better
from inv_inventory.node import DEFAULTS
from inv_inventory.queue import ScanQueue
from inv_inventory.records import make_scan_record
from inv_layout import load_layout

ROOT = pathlib.Path(__file__).resolve().parents[4]


@pytest.fixture(scope="module")
def layout():
    return load_layout("sim_small")


def tag(slot, x, y):
    return Detection(slot.location_code, "apriltag", (x, y))


def test_same_frame_nearest_label_wins(layout):
    a, b = layout.slot("A1-R-L0-B0"), layout.slot("A1-R-L0-B1")
    attr = Attributor(layout)
    item = Detection("SKU-1", "qr", (100.0, 200.0))
    result = attr.attribute_frame(10.0, [tag(a, 110, 380), tag(b, 900, 380), item])
    assert [(r.item.code, r.slot.code, r.method) for r in result] == [("SKU-1", "A1-R-L0-B0", LABEL)]


def test_location_labels_from_a_nearby_frame_count(layout):
    a = layout.slot("A2-L-L2-B2")
    attr = Attributor(layout, window_s=1.0)
    assert attr.attribute_frame(10.0, [tag(a, 480, 380)]) == []
    result = attr.attribute_frame(10.5, [Detection("SKU-2", "qr", (480, 200))])
    assert result[0].slot.code == "A2-L-L2-B2" and result[0].method == LABEL
    # ...but not once they are older than the window.
    result = attr.attribute_frame(12.0, [Detection("SKU-3", "qr", (480, 200))])
    assert result[0].method == NONE


def test_pose_fallback(layout):
    target = layout.slot("A1-L-L1-B2")
    attr = Attributor(layout, max_pose_distance_m=0.6)
    near = Detection("SKU-4", "qr", (0, 0), position_map=(target.center[0], target.center[1] - 0.3, target.center[2]))
    far = Detection("SKU-5", "qr", (0, 0), position_map=(0.0, 1.5, 1.0))
    results = attr.attribute_frame(1.0, [near, far])
    assert (results[0].slot.code, results[0].method) == ("A1-L-L1-B2", POSE)
    assert (results[1].slot, results[1].method) == (None, NONE)


def placed(code, symbology, pose, offset=(0.0, 0.0, 0.0)):
    """A detection whose map position is a layout pose plus a localization error."""
    position = tuple(p + o for p, o in zip(pose.as_tuple()[:3], offset, strict=True))
    return Detection(code, symbology, (480.0, 270.0), position_map=position)


def test_label_of_the_slot_above_does_not_claim_the_item(layout):
    # Climbing between levels: the lower item's label and the upper slot's tag are in view, the
    # lower tag is not (seen in a sim_small scan mission).
    lower, upper = layout.slot("A1-L-L0-B2"), layout.slot("A1-L-L1-B2")
    frame = [placed(upper.location_code, "apriltag", upper.location_label), placed("SKU-6", "qr", lower.item_label)]
    result = Attributor(layout).attribute_frame(1.0, frame)
    assert (result[0].slot.code, result[0].method) == ("A1-L-L0-B2", POSE)


def test_label_of_the_next_bay_does_not_claim_the_item(layout):
    item_slot, next_bay = layout.slot("A2-L-L0-B1"), layout.slot("A2-L-L0-B2")
    frame = [
        placed(next_bay.location_code, "apriltag", next_bay.location_label),
        placed("SKU-7", "qr", item_slot.item_label),
    ]
    result = Attributor(layout).attribute_frame(1.0, frame)
    assert (result[0].slot.code, result[0].method) == ("A2-L-L0-B1", POSE)


def test_own_label_wins_despite_localization_drift(layout):
    slot = layout.slot("A1-R-L1-B1")
    drift = (0.3, -0.2, 0.15)  # the same for both: they are seen in one image
    frame = [
        placed(slot.location_code, "apriltag", slot.location_label, drift),
        placed("SKU-8", "qr", slot.item_label, drift),
    ]
    result = Attributor(layout).attribute_frame(1.0, frame)
    assert (result[0].slot.code, result[0].method) == ("A1-R-L1-B1", LABEL)


def test_item_anywhere_in_the_bay_counts(layout):
    slot = layout.slot("A1-R-L0-B1")
    ax, ay = layout.rack(slot.rack_id).direction
    attr = Attributor(layout, slot_margin_m=0.1)  # 0.9 m bay: +-0.45 m, +-0.55 m with the margin
    for along, method in ((0.4, LABEL), (-0.5, LABEL), (0.6, POSE)):
        frame = [
            placed(slot.location_code, "apriltag", slot.location_label),
            placed("SKU-9", "qr", slot.item_label, (ax * along, ay * along, 0.0)),
        ]
        assert attr.attribute_frame(1.0, frame)[0].method == method, along


def test_better():
    assert better(LABEL, None) and better(LABEL, POSE) and better(POSE, NONE)
    assert not better(POSE, LABEL) and not better(LABEL, LABEL)


def _record(**kw):
    base = {
        "mission_id": "m1",
        "drone_id": "d1",
        "warehouse_id": "sim_small",
        "timestamp": dt.datetime(2026, 10, 20, 14, 3, 22, 512000, tzinfo=dt.timezone.utc),
        "code": "SKU-1",
        "symbology": "qr",
        "slot_code": "A1-R-L0-B0",
        "rack_id": "A1-R",
        "level": 0,
        "position_map": (1.0, 2.0, 0.5),
        "confidence": 1.0,
        "attribution_method": "label",
    }
    base.update(kw)
    return make_scan_record(**base)


def test_records_match_the_contract():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads((ROOT / "contracts" / "scan_record.schema.json").read_text())
    validator = jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker())
    validator.validate(_record())
    validator.validate(_record(slot_code=None, rack_id=None, level=None, position_map=None, attribution_method="none"))
    assert _record()["timestamp"] == "2026-10-20T14:03:22.512Z"


def test_queue_is_idempotent_and_tracks_sync(tmp_path):
    q = ScanQueue(tmp_path / "q.sqlite")
    r1, r2 = _record(), _record(code="SKU-2")
    assert q.put(r1) and q.put(r2)
    assert not q.put(r1)  # same scan_id twice: ignored
    assert q.counts() == (2, 2)
    q.mark_synced([r1["scan_id"]])
    assert [r["scan_id"] for r in q.pending()] == [r2["scan_id"]]
    q.mark_failed([r2["scan_id"]], "timeout")
    assert q.counts() == (2, 1)
    assert [r["code"] for r in q.records("m1")] == ["SKU-1", "SKU-2"]


def test_queue_writer_and_reader_can_overlap(tmp_path):
    path = tmp_path / "q.sqlite"
    writer, reader = ScanQueue(path), ScanQueue(path)
    errors = []

    def write():
        try:
            for i in range(200):
                writer.put(_record(code=f"SKU-{i}"))
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    t = threading.Thread(target=write)
    t.start()
    while t.is_alive():
        reader.pending(10)
    t.join()
    assert not errors
    assert reader.counts() == (200, 200)


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "inventory.yaml"
    assert yaml.safe_load(path.read_text())["inventory"]["ros__parameters"] == DEFAULTS
