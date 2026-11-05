# 0005: Gazebo Harmonic + ArduPilot SITL; Isaac Sim deferred

- Status: Accepted
- Date: 2026-10-20

## Context

Hardware (Jetson, sensors) has not arrived, so simulation comes first and must also run in CI.
Gazebo + ArduPilot SITL is ArduPilot's officially supported ROS 2 simulation path (`ardupilot_gazebo`,
`ardupilot_gz`). It runs without an NVIDIA GPU and headless.

## Decision

- **Gazebo Harmonic** (Jazzy's default pairing) with ardupilot_gazebo's ArduPilotPlugin (pinned
  commit on its `ros2` branch) and SITL (`--model JSON`, lock-step).
- Our own `sim/launch/sim_platform.launch.py` instead of ardupilot_gz's launch files: ardupilot_gz
  `main` tracks ArduPilot master (it references SITL arguments and parameter files that do not exist
  at Copter-4.7.2) and its `humble`/`jazzy` branches are stale (Nov 2024). We reuse its approach,
  not its files. Its demo launches can still be built from `deps/reference.repos`.
- Worlds are **generated** from the warehouse layout (`sim/worldgen`), deterministically.
- GPS-denied from day one: SITL GPS off, external navigation on.

## Alternatives considered

- **Isaac Sim**: photoreal and better for perception testing, but needs an RTX GPU and is heavy.
  Deferred (sim/isaac/README.md says when to add it).
- **Gazebo Jetty**: Lyrical's pairing; see ADR 0002.
- **Gazebo Classic 11**: end of life. Rejected.

## Consequences

- Cameras render in software (Mesa llvmpipe) without a GPU; CI renders only what each test needs.
- Lock-step: a slow machine slows the sim down but never breaks it.
- We carry a small SITL-only patch (rangefinder data from the JSON backend) until the ArduPilot pin
  moves past the upstream fix.

## Revisit when

Perception needs photorealism (label decoding at range, lighting, motion blur): add Isaac Sim as a
second sim platform, generating its USD world from the same layout file.
