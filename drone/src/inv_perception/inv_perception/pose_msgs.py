"""numpy rotation/translation <-> geometry_msgs."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose


def quat_from_matrix(r: np.ndarray) -> tuple[float, float, float, float]:
    t = float(np.trace(r))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def pose_from_rt(rotation: np.ndarray, translation: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in translation)
    qx, qy, qz, qw = quat_from_matrix(rotation)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    return pose
