# Pinned dependencies

The **underlay** is built once inside `docker/dev.Dockerfile` from `underlay.repos`; this repo is
the **overlay**. Everything is pinned to a tag or commit so every student and CI run builds the same
thing. Distro and image pins live in `docker/.env`.

| Pin | Value | Why this one |
|---|---|---|
| ROS 2 | Jazzy (`docker/.env`) | docs/adr/0002-ros-distro.md |
| Gazebo | Harmonic (ROS vendor packages) | Jazzy's default pairing; ADR 0005 |
| ArduPilot | tag `Copter-4.7.2` (`fb77356`) | latest stable Copter (11 Oct 2026) |
| Micro-XRCE-DDS-Gen | tag `v4.7.1` | what ArduPilot's own CI and Dockerfile use at Copter-4.7.2 (the wiki says v4.7.0) |
| ardupilot_gazebo | `ros2` branch @ `cc0290d` (2026-04-05) | Harmonic plugin + iris models, colcon package |
| micro-ROS-Agent | `jazzy` branch @ `7c93232` (2026-09-04) | the agent for our distro |
| MAVProxy | 1.8.71 (pip, Dockerfile) | what ArduPilot's CI uses at the pinned tag |
| pymavlink | from the ArduPilot submodule | dialect matches the firmware |
| pre-commit, dronecan, future | pip, Dockerfile | no apt packages (`future` is an undeclared MAVProxy dependency) |
| Isaac ROS | 4.6 base images (`docker/.env`) | Jazzy + JetPack 7.2; tags UNVERIFIED (inferred from NGC push dates) |

`ardupilot_gz` is deliberately **not** in the underlay; see `reference.repos` and ADR 0005.

## Patches (`patches/`)

Backports applied on top of the pins, each with its reason in its header:

- `ardupilot/0001-sitl-json-rangefinder-bitmask.patch`: SITL's JSON backend at Copter-4.7.2 tests
  the wrong bitmask bits for `rng_1..6`, so Gazebo rangefinder data never arrives. Backport of the
  fix on ArduPilot master. SITL-only. **Drop it** when the pin moves to a release containing it.

## Project rosdep keys

`rosdep.yaml` defines keys that rosdistro lacks (`python3-zxing-cpp`, `python3-pymavlink`). The
images register it; for a native install add `yaml file://<repo>/deps/rosdep.yaml` to
`/etc/ros/rosdep/sources.list.d/` and `rosdep update`.

## Bumping a pin

1. Change the version in `underlay.repos` (or `docker/.env`) in its own PR, with the reason.
2. ArduPilot bump: check `Tools/ros2/ros2.repos` and the CI workflows at the new tag for the
   Micro-XRCE-DDS-Gen version; re-check docs/ardupilot.md (AP_DDS topics/QoS, `/ap/tf` and
   `/ap/cmd_vel` handling, EKF origin) against the new source; run `just params-check` (renamed
   parameters fail it); drop patches that are upstream.
3. `just image` locally, then `just build test sim-smoke`.
4. CI (`ci`, `sim-smoke`) must pass; `docker-publish` republishes the image after merge.
