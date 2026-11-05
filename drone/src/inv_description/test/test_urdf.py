"""The URDF expands, parses, and contains every frame named in docs/interfaces.md."""

import math

import pytest

from inv_description.extrinsics import all_frame_poses, expand_xacro, frame_pose

# Frames other packages and the sim rely on (docs/interfaces.md, "Frames").
REQUIRED_FRAMES = [
    "base_link",
    "fc_imu_link",
    "camera_link",
    "camera_left_optical_frame",
    "camera_right_optical_frame",
    "camera_depth_optical_frame",
    "camera_color_optical_frame",
    "camera_imu_optical_frame",
    "rangefinder_link",
    "flow_link",
]


@pytest.fixture(scope="module")
def urdf_xml():
    return expand_xacro()


def test_urdf_parses(urdf_xml):
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string(urdf_xml)
    assert robot.name == "inv_quad"
    assert robot.get_root() == "base_link"
    names = {link.name for link in robot.links}
    for frame in REQUIRED_FRAMES:
        assert frame in names, frame


def test_every_frame_reaches_base_link(urdf_xml):
    poses = all_frame_poses(urdf_xml)
    for frame in REQUIRED_FRAMES[1:]:
        assert frame in poses


def test_optical_frames_look_forward(urdf_xml):
    # An optical frame's +z must point along the airframe's +x (camera looks forward).
    m = frame_pose(urdf_xml, "camera_color_optical_frame").matrix
    assert m[:3, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)
    # +x of the optical frame points to the right of the drone (-y in base_link).
    assert m[:3, 0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)


def test_stereo_pair_is_symmetric(urdf_xml):
    centre = frame_pose(urdf_xml, "camera_link").xyz
    left = frame_pose(urdf_xml, "camera_left").xyz
    right = frame_pose(urdf_xml, "camera_right").xyz
    assert left[0] == pytest.approx(right[0])
    assert left[1] - centre[1] == pytest.approx(-(right[1] - centre[1]))
    assert left[1] > right[1]  # left imager is on the drone's left (+y)


def test_rangefinder_points_down(urdf_xml):
    m = frame_pose(urdf_xml, "rangefinder_link").matrix
    assert m[:3, 0] == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)
    assert frame_pose(urdf_xml, "rangefinder_link").rpy[1] == pytest.approx(math.pi / 2)
