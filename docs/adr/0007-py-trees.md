# 0007: py_trees for the mission; routes generated from the layout

- Status: Accepted
- Date: 2026-10-20

## Context

The mission is a fixed sequence with failsafes: wait until ready, take off, visit every scan
point, land; abort on safety events. It is not performance-critical. Contributors rotate every
semester and most know Python better than C++.

## Decision

- **py_trees / py_trees_ros** behaviour trees (`inv_mission`), driving the `/flight/*` actions.
- **Routes are generated** from the warehouse layout: aisle → rack face → shelf level → hover point
  per slot, serpentine per level, entering and leaving aisles at their start end. Not hand-flown,
  not free-planned.
- **Stop and hover** at every scan point (motion blur and timing problems mostly disappear).

## Alternatives considered

- **BehaviorTree.CPP v4 + Groot2**: what Nav2 uses, with a good visual editor. Faster and more
  widely used in industry, but C++ and XML trees slow down rotating students for a layer that does
  not need speed. Reconsider if the mission grows complex enough to need Groot2's tooling.
- **A hand-written state machine**: fine today, but retries, rescans and failsafe branches
  (phase 2) are clearer as trees.
- **Scanning while moving**: faster missions, but motion blur and timestamp alignment. Deferred.

## Consequences

- `ros2 run inv_mission route <warehouse>` prints the route without flying it; unit tests check it
  covers every slot and keeps the camera on target.
- `/mission/status` doubles as the heartbeat the flight interface watches.

## Revisit when

The tree needs many reactive branches or other teams want Groot2 (switch to BehaviorTree.CPP), or
mission time becomes the bottleneck (scan while moving).
