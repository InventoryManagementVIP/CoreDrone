import math
import pathlib

import numpy as np
import pytest
import yaml

from inv_perception import code_reader, tag_adapter
from inv_perception.code_reader import decode, symbology_for
from inv_perception.geometry import solve_label_pose
from inv_perception.pose_msgs import pose_from_rt
from inv_perception.tag_adapter import tag_code


def qr_image(payload: str, px_per_module: int = 8) -> np.ndarray:
    import qrcode

    qr = qrcode.QRCode(box_size=px_per_module, border=4)
    qr.add_data(payload)
    qr.make(fit=True)
    img = np.array(qr.make_image(fill_color="black", back_color="white").convert("L"))
    return np.pad(img, 60, constant_values=255)


def test_decode_qr():
    results = decode(qr_image("SKU-01234"), ["QRCode"])
    assert [r["text"] for r in results] == ["SKU-01234"]
    assert results[0]["format"] == "QRCode"
    assert results[0]["corners"].shape == (4, 2)


def test_decode_ignores_formats_not_asked_for():
    assert decode(qr_image("SKU-01234"), ["Code128"]) == []


def test_symbology_names_follow_the_contract():
    assert symbology_for("QRCode") == "qr"
    assert symbology_for("Code128") == "code128"
    assert symbology_for("EAN13") == "ean13"
    assert symbology_for("Aztec") == "other"


def test_tag_code():
    assert tag_code("36h11", 5) == "tag36h11:5"
    assert tag_code("tag36h11", 17) == "tag36h11:17"


F, CX, CY = 700.0, 480.0, 270.0
K = np.array([[F, 0, CX], [0, F, CY], [0, 0, 1.0]])
# A label facing the camera (its z axis back along the optical axis, its y axis up in the image).
FACING = np.diag([1.0, -1.0, -1.0])


def project(rot: np.ndarray, t: np.ndarray, size: float) -> np.ndarray:
    from inv_perception.geometry import square_object_points

    cam = (rot @ square_object_points(size).T).T + t
    return np.stack([CX + F * cam[:, 0] / cam[:, 2], CY + F * cam[:, 1] / cam[:, 2]], axis=1)


def test_pnp_recovers_distance_head_on():
    # Head-on is the degenerate case for planar PnP (two mirror poses fit), so only check position.
    t_true = np.array([0.0, 0.0, 1.5])
    _, t = solve_label_pose(project(FACING, t_true, 0.2), 0.2, K)
    assert t == pytest.approx(t_true, abs=1e-6)


def test_pnp_recovers_a_tilted_pose():
    a = math.radians(30.0)  # label turned 30 deg about the camera's vertical axis
    tilt = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    rot_true, t_true = tilt @ FACING, np.array([0.1, -0.05, 1.5])
    rot, t = solve_label_pose(project(rot_true, t_true, 0.2), 0.2, K)
    assert t == pytest.approx(t_true, abs=1e-6)
    assert rot == pytest.approx(rot_true, abs=1e-6)
    q = pose_from_rt(rot, t).orientation
    assert math.isclose(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, abs_tol=1e-9)


@pytest.mark.parametrize(
    ("module", "node", "file"),
    [(code_reader, "code_reader", "code_reader.yaml"), (tag_adapter, "tag_adapter", "tag_adapter.yaml")],
)
def test_configs_match_defaults(module, node, file):
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / file
    assert yaml.safe_load(path.read_text())[node]["ros__parameters"] == module.DEFAULTS
