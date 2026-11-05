# Architecture Decision Records

Short documents that record *why* the repo is the way it is, so the next semester does not have to
rediscover it. One decision per file; never rewrite history: supersede a decision with a new ADR
and mark the old one `Superseded by NNNN`.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-monorepo.md) | Monorepo with hard boundaries; companion app stays in its own repo for now | Accepted |
| [0002](0002-ros-distro.md) | ROS 2 Jazzy, as one build argument | Accepted (pending Jetson delivery) |
| [0003](0003-ardupilot-dds.md) | ArduPilot Copter + AP_DDS, behind one flight-interface package | Accepted |
| [0004](0004-isaac-vslam-swappable-localization.md) | Isaac ROS VSLAM behind a swappable localization interface | Accepted |
| [0005](0005-gazebo-harmonic.md) | Gazebo Harmonic + ArduPilot SITL; Isaac Sim deferred | Accepted |
| [0006](0006-label-based-slot-attribution.md) | Slot attribution from location labels, pose as fallback | Accepted |
| [0007](0007-py-trees.md) | py_trees for the mission; routes generated from the layout | Accepted |
| [0008](0008-store-and-forward-uplink.md) | Store-and-forward uplink through SQLite | Accepted |
| [0009](0009-hardware-baseline.md) | Hardware baseline: Pixhawk 6C, Orin Nano 8 GB, OAK-D Lite, X500 V2 | Accepted |

## Template

```markdown
# NNNN: Title

- Status: Proposed | Accepted | Superseded by NNNN
- Date: YYYY-MM-DD
- Deciders: who

## Context
What forces are at play, and what we know (with sources).

## Decision
What we do.

## Alternatives considered
Each with why it lost.

## Consequences
Good and bad.

## Revisit when
The concrete trigger that should reopen this decision.
```
