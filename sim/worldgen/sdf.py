"""Tiny helpers for building SDF with ElementTree (so values are always escaped correctly)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Iterable


def fmt(value: float) -> str:
    # 9 significant digits keep angles exact to ~1e-8 rad (6 would turn pi/2 into 1.5708);
    # adding 0.0 turns -0.0 into 0.0.
    return f"{value + 0.0:.9g}"


def vec(values: Iterable[float]) -> str:
    return " ".join(fmt(v) for v in values)


def sub(parent: ET.Element, tag: str, text: str | None = None, **attrib: str) -> ET.Element:
    el = ET.SubElement(parent, tag, {k.rstrip("_"): v for k, v in attrib.items()})
    if text is not None:
        el.text = text
    return el


def pose(
    parent: ET.Element,
    x: float,
    y: float,
    z: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    relative_to: str | None = None,
) -> ET.Element:
    attrib = {"relative_to": relative_to} if relative_to else {}
    return sub(parent, "pose", vec((x, y, z, roll, pitch, yaw)), **attrib)


def material(parent: ET.Element, rgba: tuple[float, float, float, float], albedo_map: str | None = None) -> ET.Element:
    m = sub(parent, "material")
    sub(m, "ambient", vec(rgba))
    sub(m, "diffuse", vec(rgba))
    sub(m, "specular", "0.05 0.05 0.05 1")
    if albedo_map:
        metal = sub(sub(m, "pbr"), "metal")
        sub(metal, "albedo_map", albedo_map)
        sub(metal, "metalness", "0")
        sub(metal, "roughness", "1")
    return m


def box_visual(
    link: ET.Element,
    name: str,
    size: tuple[float, float, float],
    xyz_yaw: tuple[float, float, float, float],
    rgba: tuple[float, float, float, float],
    collision: bool = True,
) -> None:
    x, y, z, yaw = xyz_yaw
    v = sub(link, "visual", name=name)
    pose(v, x, y, z, yaw=yaw)
    sub(sub(sub(v, "geometry"), "box"), "size", vec(size))
    material(v, rgba)
    if collision:
        c = sub(link, "collision", name=f"{name}_collision")
        pose(c, x, y, z, yaw=yaw)
        sub(sub(sub(c, "geometry"), "box"), "size", vec(size))


def to_string(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def model_config(name: str, description: str) -> str:
    root = ET.Element("model")
    sub(root, "name", name)
    sub(root, "version", "1.0")
    sub(root, "sdf", "model.sdf", version="1.9")
    sub(root, "description", description)
    return to_string(root)
