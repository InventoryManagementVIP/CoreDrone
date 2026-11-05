# 0006: Slot attribution from location labels, pose as fallback

- Status: Accepted
- Date: 2026-10-20

## Context

The original plan mapped each barcode to a rack slot using the SLAM pose at scan time. VSLAM
drifts, warehouse aisles are repetitive and low-texture, and slots are often less than 1 m apart,
so pose-only attribution will put items in the wrong slot. Real warehouses already put location
labels on rack beams.

## Decision

- Every slot has a location label on the beam under it: the warehouse's own location barcodes,
  or AprilTags (tag36h11) where there are none. The layout file maps tag IDs / location codes to
  slots.
- Perception reports every code it decodes without interpreting it. Inventory classifies codes
  with the layout (location vs item) and attributes each item to the slot whose location label is
  **nearest in the same image**, or in images from the last second (the drone hovers).
- A location label only counts if the item lies inside that label's slot opening (within the bay
  width, between the deck and the beam above, 0.1 m slack), measured relative to where the label
  was seen. Both positions come from the same image, so VSLAM drift cancels out. Added after the
  first sim_small scan mission: while the drone climbed between levels or moved between bays, the
  only label in view could belong to the neighbouring slot (2 of 31 items in the wrong slot).
- Only when no plausible location label was seen does it fall back to the SLAM pose (nearest slot
  centre within 0.6 m), and records `attribution_method: pose`. Otherwise `none`.
- The route puts the camera 1.0 m from each slot, so at a hover point one slot's labels are in
  view; in transit, neighbours' labels are (hence the check above).

## Alternatives considered

- **Pose-only attribution**: simple, but wrong under drift and with dense slots. Rejected as the
  primary method.
- **Box detection without labels**: phase 2 at the earliest.

## Consequences

- Attribution quality depends on label reads, which `sim/evaluation` measures (correct-slot rate).
- Rack AprilTags can later double as absolute position fixes for localization.
- Real deployments need a location label or tag on every slot (a warehouse survey task).

## Revisit when

A warehouse cannot be labelled, or the correct-slot rate from labels drops below what pose
attribution achieves with the real VSLAM drift.
