"""Read sensor extrinsics out of the URDF, so nothing else has to hard-code them.

Used by the sim drone model generator (sim/models/generate_drone_model.py) and by tests.
`ros2 run inv_description print_extrinsics` prints every frame's pose in base_link.
"""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

ROOT_FRAME = "base_link"


@dataclass(frozen=True)
class FramePose:
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    @property
    def matrix(self) -> np.ndarray:
        return _matrix(self.xyz, self.rpy)


def default_xacro_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("inv_description")
        return os.path.join(share, "urdf", "inv_quad.urdf.xacro")
    except Exception:  # noqa: BLE001 - fall back to the source tree outside a ROS environment
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(os.path.dirname(here), "urdf", "inv_quad.urdf.xacro")


def expand_xacro(path: str | None = None) -> str:
    import xacro

    doc = xacro.process_file(path or default_xacro_path())
    return doc.toxml()


def _rot(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF rpy: fixed-axis roll about x, then pitch about y, then yaw about z (R = Rz Ry Rx).
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _matrix(xyz, rpy) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _rot(rpy)
    m[:3, 3] = xyz
    return m


def _rpy_from_matrix(m: np.ndarray) -> tuple[float, float, float]:
    r = m[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: put everything in yaw
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    return (roll, pitch, yaw)


def _floats(text: str | None) -> tuple[float, float, float]:
    if not text:
        return (0.0, 0.0, 0.0)
    a, b, c = (float(v) for v in text.split())
    return (a, b, c)


def joint_tree(urdf_xml: str) -> dict[str, tuple[str, np.ndarray]]:
    """child link -> (parent link, parent_T_child)."""
    tree: dict[str, tuple[str, np.ndarray]] = {}
    for joint in ET.fromstring(urdf_xml).findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        origin = joint.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None)
        rpy = _floats(origin.get("rpy") if origin is not None else None)
        tree[child] = (parent, _matrix(xyz, rpy))
    return tree


def frame_pose(urdf_xml: str, frame: str, root: str = ROOT_FRAME) -> FramePose:
    """Pose of `frame` in `root`, composed along the joint chain."""
    tree = joint_tree(urdf_xml)
    m = np.eye(4)
    current = frame
    while current != root:
        if current not in tree:
            raise KeyError(f"frame {frame!r} is not connected to {root!r}")
        parent, t = tree[current]
        m = t @ m
        current = parent
    return FramePose(tuple(float(v) for v in m[:3, 3]), _rpy_from_matrix(m))


def all_frame_poses(urdf_xml: str, root: str = ROOT_FRAME) -> dict[str, FramePose]:
    return {child: frame_pose(urdf_xml, child, root) for child in joint_tree(urdf_xml)}


def main(argv: list[str] | None = None) -> int:
    xml = expand_xacro(argv[0] if argv else None)
    for name, pose in sorted(all_frame_poses(xml).items()):
        x, y, z = pose.xyz
        r, p, yw = (math.degrees(v) for v in pose.rpy)
        print(f"{name:28s} xyz=({x:+.3f}, {y:+.3f}, {z:+.3f})  rpy_deg=({r:+.1f}, {p:+.1f}, {yw:+.1f})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
