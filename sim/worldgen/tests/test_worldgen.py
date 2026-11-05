"""World generator unit tests. Run with `just test-sim` (or pytest sim/worldgen/tests)."""

from __future__ import annotations

import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM.parent / "drone" / "src" / "inv_layout"))  # works without a ROS build too

from inv_layout import load_layout  # noqa: E402  (importable thanks to worldgen/__init__)
from worldgen import labels  # noqa: E402
from worldgen.__main__ import generate  # noqa: E402
from worldgen.warehouse import assign_items  # noqa: E402


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    return generate("sim_small", "label", tmp_path_factory.mktemp("worlds"), with_drone=False)


def test_items_are_deterministic():
    layout = load_layout("sim_small")
    assert assign_items(layout) == assign_items(layout)
    filled = [v for v in assign_items(layout).values() if v]
    assert 0.6 * len(layout.slots) < len(filled) < len(layout.slots)
    assert len(set(filled)) == len(filled), "item codes must be unique"


def test_outputs_exist(generated):
    for name in ("world.sdf", "bridge.yaml", "ground_truth.json", "info.json"):
        assert (generated / name).is_file(), name
    assert (generated / "models" / "warehouse_sim_small" / "model.sdf").is_file()


def test_ground_truth_matches_layout(generated):
    layout = load_layout("sim_small")
    gt = json.loads((generated / "ground_truth.json").read_text())
    assert [s["slot_code"] for s in gt["slots"]] == [s.code for s in layout.slots]
    for item, slot_code in gt["items"].items():
        entry = next(s for s in gt["slots"] if s["slot_code"] == slot_code)
        assert entry["item_code"] == item


def test_world_and_model_are_valid_sdf_xml(generated):
    world = ET.parse(generated / "world.sdf").getroot()
    assert world.find("world").get("name") == "sim_small"
    model = ET.parse(generated / "models" / "warehouse_sim_small" / "model.sdf").getroot()
    visuals = model.findall("./model/link/visual")
    gt = json.loads((generated / "ground_truth.json").read_text())
    loc_labels = [v for v in visuals if v.get("name").startswith("label_loc_")]
    item_labels = [v for v in visuals if v.get("name").startswith("label_item_")]
    assert len(loc_labels) == len(gt["slots"])
    assert len(item_labels) == len(gt["items"])
    for v in loc_labels + item_labels:
        uri = v.find("./material/pbr/metal/albedo_map").text
        texture = generated / "models" / uri.removeprefix("model://")
        assert texture.is_file(), uri


def test_label_geometry_is_on_the_rack_face(generated):
    layout = load_layout("sim_small")
    slot = layout.slot("A1-R-L0-B0")
    model = ET.parse(generated / "models" / "warehouse_sim_small" / "model.sdf").getroot()
    v = next(v for v in model.iter("visual") if v.get("name") == f"label_loc_{slot.code}")
    x, y, z, _, _, yaw = (float(t) for t in v.find("pose").text.split())
    assert (x, y, z) == pytest.approx(slot.location_label.as_tuple()[:3], abs=1e-4)
    assert yaw == pytest.approx(slot.location_label.yaw, abs=1e-4)


def test_apriltag_texture_matches_its_id():
    cv2 = pytest.importorskip("cv2")
    img = np.array(labels.apriltag_label(17, px_per_cell=12).image)
    img = np.pad(img, 40, constant_values=255)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(img)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(img, dictionary)
    assert ids is not None and ids.flatten().tolist() == [17]


def test_qr_texture_decodes():
    payload = "SKU-01234"
    img = np.array(labels.qr_label(payload).image)
    if importlib.util.find_spec("zxingcpp"):
        import zxingcpp

        results = zxingcpp.read_barcodes(img)
        assert [r.text for r in results] == [payload]
    else:
        cv2 = pytest.importorskip("cv2")
        text, _, _ = cv2.QRCodeDetector().detectAndDecode(img)
        assert text == payload


def test_label_scale():
    qr = labels.qr_label("SKU-01234")
    assert qr.label_scale == pytest.approx((21 + 8) / 21)  # version 1 symbol + 4-module quiet zone
    assert labels.apriltag_label(0).label_scale == pytest.approx(10 / 8)
    with pytest.raises(ValueError):
        labels.apriltag_label(587)


@pytest.mark.skipif(
    importlib.util.find_spec("xacro") is None or importlib.util.find_spec("inv_description") is None,
    reason="needs the ROS environment (run inside the dev container after `just build`)",
)
def test_drone_model_sensor_poses_come_from_the_urdf(tmp_path):
    from inv_description.extrinsics import expand_xacro, frame_pose
    from worldgen.drone_model import BASE, build_drone_model, load_sensor_config

    try:
        bridge = build_drone_model(expand_xacro(), "full", tmp_path)
    except Exception as exc:  # noqa: BLE001 - ardupilot_gazebo missing outside the image
        pytest.skip(f"ardupilot_gazebo not available: {exc}")
    urdf = expand_xacro()
    model = ET.parse(tmp_path / "inv_quad" / "model.sdf").getroot().find("model")
    config = load_sensor_config()
    for name in config["flight_sensors"] + config["sensor_sets"]["full"]:
        link = model.find(f"./link[@name='{name}_sensor_link']")
        assert link is not None, name
        pose = link.find("pose")
        assert pose.get("relative_to") == BASE
        values = [float(v) for v in pose.text.split()]
        expected = frame_pose(urdf, config["sensors"][name]["link"])
        assert values[:3] == pytest.approx(list(expected.xyz), abs=1e-6), name
        assert values[3:] == pytest.approx(list(expected.rpy), abs=1e-6), name
    # The rangefinder reaches ArduPilot through the plugin, and every sensor has a bridge entry.
    plugin = model.find("./plugin[@name='ArduPilotPlugin']")
    assert plugin.find("./sensor/type").text == "RANGEFINDER"
    ros_topics = {b["ros_topic_name"] for b in bridge}
    expected_topics = {"/clock", "/sim/ground_truth/odom", "/camera/color/image_raw", "/camera/left/image_raw"}
    assert expected_topics | {"/camera/imu", "/rangefinder/scan"} <= ros_topics
