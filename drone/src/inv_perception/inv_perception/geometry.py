"""Label pose from its four corners (PnP), shared by the CPU decoders."""

from __future__ import annotations

import numpy as np


def square_object_points(size_m: float) -> np.ndarray:
    """Corners of a square label of edge `size_m` in the label frame: x right, y up, z out of the
    label towards the viewer (OpenCV ArUco convention).

    Order is top-left, top-right, bottom-right, bottom-left, which is how decoders report the
    corners of an upright label.
    """
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def solve_label_pose(
    corners_px: np.ndarray, size_m: float, camera_matrix: np.ndarray, dist_coeffs: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray] | None:
    """Return (rotation matrix, translation) of the label in the camera optical frame, or None."""
    import cv2

    if size_m <= 0.0 or corners_px.shape != (4, 2):
        return None
    ok, rvec, tvec = cv2.solvePnP(
        square_object_points(size_m),
        corners_px.astype(np.float64),
        camera_matrix,
        dist_coeffs if dist_coeffs is not None else np.zeros(5),
        # SQPNP is globally optimal; IPPE_SQUARE (OpenCV 4.6) returned poses a few cm and degrees off
        # on exact synthetic data (test/test_decode.py).
        flags=cv2.SOLVEPNP_SQPNP,
    )
    if not ok:
        return None
    rot, _ = cv2.Rodrigues(rvec)
    return rot, tvec.reshape(3)
