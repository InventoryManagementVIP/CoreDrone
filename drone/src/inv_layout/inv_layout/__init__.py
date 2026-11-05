"""Warehouse layout model shared by the mission, the inventory node and the sim world generator.

One layout file (config/warehouses/<name>.yaml) drives everything that needs to know where racks,
slots and labels are. Keep this package free of ROS runtime dependencies so that sim/worldgen and
plain-Python tests can use it too.
"""

from inv_layout.layout import (
    Aisle,
    GeodeticOrigin,
    LabelSpec,
    Layout,
    Pose,
    Rack,
    RackGeometry,
    ScanSettings,
    SimSettings,
    Slot,
    body_pose_for_camera,
    camera_hover_pose,
)
from inv_layout.loader import LayoutError, load_layout, parse_layout, validate_layout_dict
from inv_layout.paths import layout_schema_path, repo_root, warehouse_path

__all__ = [
    "Aisle",
    "GeodeticOrigin",
    "LabelSpec",
    "Layout",
    "LayoutError",
    "Pose",
    "Rack",
    "RackGeometry",
    "ScanSettings",
    "SimSettings",
    "Slot",
    "body_pose_for_camera",
    "camera_hover_pose",
    "layout_schema_path",
    "load_layout",
    "parse_layout",
    "repo_root",
    "validate_layout_dict",
    "warehouse_path",
]
