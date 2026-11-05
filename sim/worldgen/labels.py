"""Label textures: QR codes for items, AprilTag tag36h11 for location labels.

Layout `size_m` values are the code itself (QR symbol, AprilTag black square). The printed label
is larger because it includes the white quiet zone; `label_scale` is printed edge / code edge.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

QR_QUIET_ZONE_MODULES = 4  # required by the QR specification
APRILTAG_WHITE_BORDER_CELLS = 1


@dataclass(frozen=True)
class LabelImage:
    image: Image.Image
    label_scale: float  # printed label edge / code edge


def qr_label(payload: str, px_per_module: int = 12) -> LabelImage:
    import qrcode

    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=px_per_module,
        border=QR_QUIET_ZONE_MODULES,
    )
    qr.add_data(payload)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert("L")
    modules = qr.modules_count
    return LabelImage(img, (modules + 2 * QR_QUIET_ZONE_MODULES) / modules)


def _aruco_apriltag_36h11(tag_id: int, inner_px: int) -> np.ndarray:
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "generateImageMarker"):  # OpenCV >= 4.7
        return cv2.aruco.generateImageMarker(dictionary, tag_id, inner_px, borderBits=1)
    return cv2.aruco.drawMarker(dictionary, tag_id, inner_px, borderBits=1)  # OpenCV 4.6 (Ubuntu 24.04)


def apriltag_label(tag_id: int, px_per_cell: int = 16) -> LabelImage:
    """tag36h11: 6x6 data cells + 1 black border cell + 1 white border cell = 10 x 10 cells."""
    if not 0 <= tag_id <= 586:
        raise ValueError(f"tag36h11 has IDs 0..586, got {tag_id}")
    inner = _aruco_apriltag_36h11(tag_id, 8 * px_per_cell)
    border = APRILTAG_WHITE_BORDER_CELLS * px_per_cell
    img = np.full((inner.shape[0] + 2 * border, inner.shape[1] + 2 * border), 255, dtype=np.uint8)
    img[border:-border, border:-border] = inner
    return LabelImage(Image.fromarray(img), 10.0 / 8.0)
