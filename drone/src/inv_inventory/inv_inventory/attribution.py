"""Slot attribution: which rack slot does an item read belong to?

Phase 1 rule (docs/adr/0006-label-based-slot-attribution.md):
  1. LABEL: a location label/tag (from the warehouse layout) seen in the same frame -> the one
     nearest to the item in the image. If none in this frame, use location labels seen in the last
     `window_s` seconds (the drone hovers, so the view barely changes). When both have a 3D
     position, the label only counts if the item lies inside that label's slot opening (see
     `_inside_slot`): while the drone climbs between levels or moves between bays, the only
     location label in view can belong to the neighbouring slot.
  2. POSE: otherwise, the slot whose centre is nearest to the item's map position, if within
     `max_pose_distance_m`. VSLAM drift makes this the weaker fallback.
  3. NONE: record the read without a slot.

Within a mission an item code is one box per slot (`worth_recording`): a barcode names a product,
so the same code in two slots is two boxes. Only a LABEL attribution can establish a slot, since
a POSE fallback can land in the neighbouring slot through VSLAM drift. Boxes of one product side
by side in one slot are counted once.
TODO(phase-2): confidence fusion, and later corrections superseding records already queued.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from inv_layout import Layout, Slot

LABEL, POSE, NONE = "label", "pose", "none"
_RANK = {NONE: 0, POSE: 1, LABEL: 2}


@dataclass(frozen=True)
class Detection:
    code: str
    symbology: str
    center_px: tuple[float, float]
    confidence: float = 1.0
    position_map: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class Attribution:
    item: Detection
    slot: Slot | None
    method: str


class Attributor:
    def __init__(
        self, layout: Layout, window_s: float = 1.0, max_pose_distance_m: float = 0.6, slot_margin_m: float = 0.1
    ):
        self.layout = layout
        self.window_s = window_s
        self.max_pose_distance_m = max_pose_distance_m
        self.slot_margin_m = slot_margin_m
        self.locations = layout.slots_by_location_code()
        # Per slot: unit vector along the rack face, and the opening's (width, height).
        self._openings = {s.code: (layout.rack(s.rack_id).direction, layout.slot_opening(s)) for s in layout.slots}
        self._recent: deque[tuple[float, Detection, Slot]] = deque()

    def is_location(self, det: Detection) -> bool:
        return det.code in self.locations

    def attribute_frame(self, stamp_s: float, detections: list[Detection]) -> list[Attribution]:
        locations = [(d, self.locations[d.code]) for d in detections if d.code in self.locations]
        items = [d for d in detections if d.code not in self.locations]
        for det, slot in locations:
            self._recent.append((stamp_s, det, slot))
        while self._recent and stamp_s - self._recent[0][0] > self.window_s:
            self._recent.popleft()
        candidates = locations or [(d, s) for _, d, s in self._recent]
        return [self._attribute(item, candidates) for item in items]

    def _attribute(self, item: Detection, candidates: list[tuple[Detection, Slot]]) -> Attribution:
        plausible = [(label, slot) for label, slot in candidates if self._inside_slot(item, label, slot)]
        if plausible:
            _, slot = min(plausible, key=lambda c: math.dist(c[0].center_px, item.center_px))
            return Attribution(item, slot, LABEL)
        if item.position_map is not None:
            slot, dist = self.layout.nearest_slot(*item.position_map)
            if dist <= self.max_pose_distance_m:
                return Attribution(item, slot, POSE)
        return Attribution(item, None, NONE)

    def _inside_slot(self, item: Detection, label: Detection, slot: Slot) -> bool:
        """Does the item label lie inside the opening of the slot that `label` marks?

        Measured relative to where the location label was seen (both come from the same image, or
        images a moment apart, so localization drift cancels out): within the bay along the rack
        face, and between the slot's deck and the beam above, each with `slot_margin_m` to spare.
        Without both positions there is nothing to check, and the label counts.
        """
        if item.position_map is None or label.position_map is None:
            return True
        # The slot's face point (deck height, middle of the bay), shifted to where the label was seen.
        expected_label = slot.location_label.as_tuple()[:3]
        face = [seen + f - e for seen, f, e in zip(label.position_map, slot.face_point, expected_label, strict=True)]
        dx, dy, dz = (i - f for i, f in zip(item.position_map, face, strict=True))
        (ax, ay), (width, height) = self._openings[slot.code]
        m = self.slot_margin_m
        return abs(dx * ax + dy * ay) <= width / 2 + m and -m <= dz <= height + m


def worth_recording(a: Attribution, recorded: dict[str | None, str]) -> bool:
    """Is this read of an item code worth a scan record, given that code's records this mission?

    `recorded` maps slot code (None for no slot) -> attribution method already recorded there.
    A LABEL read counts once per slot. A weaker read only counts while the code has nothing better
    anywhere: then it is most likely the same box, seen from a worse angle.
    """
    if a.method == LABEL:
        return recorded.get(a.slot.code) != LABEL
    if a.method == POSE:
        return all(slot is None for slot in recorded)
    return not recorded
