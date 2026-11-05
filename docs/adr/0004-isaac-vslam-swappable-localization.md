# 0004: Isaac ROS Visual SLAM behind a swappable localization interface

- Status: Accepted
- Date: 2026-10-20

## Context

The previous attempt stalled on ORB-SLAM3 and then Stella VSLAM, largely because they had to be
built from source and broke on macOS. The team's documents disagreed: the midterm deck and process
doc said Stella VSLAM; the latest meeting notes and the architecture diagram said Isaac ROS VSLAM +
nvblox. Isaac ROS (cuVSLAM) is GPU-accelerated on the Jetson, ships as containers and fuses stereo
with IMU, but it needs CUDA: it cannot run on Macs, most laptops or GitHub-hosted CI.

## Decision

Localization is an **interface**, not an implementation. Every backend publishes TF
`map → odom → base_link` (REP-105) and `nav_msgs/Odometry` on `/localization/odom`; a shared
adapter turns that into external navigation for the FC and reports health on
`/localization/status`. Backends, chosen with `localization:=`:

- `isaac`: Isaac ROS Visual SLAM on the Jetson or an RTX machine (config + launch; needs CUDA)
- `groundtruth`: sim only; Gazebo's true pose with optional seeded noise, drift and dropouts. What
  Mac/Windows/CI developers use.
- `rtabmap`: CPU stereo + IMU fallback, apt-installable. Stub until needed.

Obstacle mapping follows the same pattern (`mapping:=nvblox|none`); phase 1 only uses nvblox for a
"corridor ahead blocked → hold" check, no 3D path planning.

## Alternatives considered

- **ORB-SLAM3 / Stella VSLAM** (vendored, built from source): rejected. This is exactly what sank
  the previous repo, and neither fuses an IMU as well on Jetson hardware.
- **Isaac only, no interface**: nobody without an NVIDIA GPU could run the stack, and CI could not
  test anything downstream of localization. Rejected.
- **rtabmap as primary**: CPU-only and slower; kept as the fallback for a real-sensor path without
  a GPU.

## Consequences

- Everyone can run the full mission in sim with `groundtruth`; noise/drift/dropout parameters let
  them rehearse VSLAM error and the localization-lost failsafe.
- The Isaac path is not exercised in CI; a manual GPU check is documented in
  docs/onboarding/linux-nvidia.md.
- Camera choice matters: VSLAM wants global-shutter stereo with a hardware-synced IMU at modest
  resolution (640-1280 px, 30-60 fps), not "4K". See real/sensors/README.md.

## Revisit when

Isaac ROS 5.x (which renames the package to `isaac_ros_cuvslam`) is adopted with Lyrical, or a
GPU-less real-sensor path is needed (implement `rtabmap`).
