# 0002: ROS 2 Jazzy, as one build argument

- Status: Accepted, pending confirmation when the Jetson arrives
- Date: 2026-10-20

## Context

As of October 2026:

- **Humble** reaches end of life in **May 2027**, seven months away. **Jazzy** is supported until
  **May 2029**. **Lyrical** (May 2026) until May 2031.
- The deciding constraint is Isaac ROS on the Jetson we will actually get, and the model is not
  known yet. But it no longer matters which Jetson it is: **Isaac ROS 4.6** (18 Aug 2026) "Added
  support for Jetson Orin" and "Added support for JetPack 7.2", and **JetPack 7.2** (Jetson Linux
  39.2, Ubuntu 24.04) covers the entire Orin family (AGX Orin, Orin NX, Orin Nano) and Thor.
  Isaac ROS 4.x is designed and tested for ROS 2 Jazzy. Isaac ROS 3.2 (Humble, JetPack 6) still
  exists but is the old line.
- **Isaac ROS 5.0** (22 Sep 2026) moved to ROS 2 Lyrical, removed the NITROS packages and renamed
  `isaac_ros_visual_slam` to `isaac_ros_cuvslam`.
- **Gazebo:** Harmonic is Jazzy's default pairing (binary `ros-jazzy-ros-gz`). Lyrical pairs with
  Gazebo Jetty. Our simulator decision (ADR 0005) is Harmonic.
- **ArduPilot:** its ROS 2 docs still say "Humble is the only version supported", and its CI at
  Copter-4.7.2 tests ROS 2 on Humble only. AP_DDS itself speaks XRCE-DDS, which is distro-agnostic;
  its README already assumes Jazzy for IDL work.

We verified the Jazzy path end to end in the dev image: ardupilot_msgs, ardupilot_sitl (Copter-4.7.2
with DDS), ardupilot_gazebo and micro-ROS-Agent (`jazzy` branch) all build, and SITL's AP_DDS talks
to the stack through the agent.

## Decision

**ROS 2 Jazzy**, set once as `ROS_DISTRO` in `docker/.env`. Every Dockerfile takes it as a build
argument without a default, and every CI job reads it from that file. `humble` remains a working
branch in `docker/scripts/install-apt-deps.sh` (Harmonic from packages.osrfoundation.org) so the
switch stays a one-line change, but CI does not exercise it.

## Alternatives considered

- **Humble**: ArduPilot's official choice, but end of life in May 2027 forces a migration within a
  year, Harmonic is a non-default pairing on it, and Isaac ROS's current line is Jazzy. Rejected.
- **Lyrical**: longest support, but it pairs with Gazebo Jetty (breaks ADR 0005), Isaac ROS 5.0 is
  one month old with renamed packages, and neither ArduPilot nor ardupilot_gazebo supports it yet.
  Deferred: plan the Jazzy → Lyrical migration for 2027-2028, well before Jazzy's end of life.

## Consequences

- One distro across dev image, Isaac images, Jetson image and CI.
- We run ArduPilot's ROS 2 packages on a distro ArduPilot does not officially test; our own CI
  (`sim-smoke`, `params-check`) is what proves it on every relevant PR.
- The Jetson must run JetPack 7.2.

## Revisit when

- The delivered Jetson or its carrier board cannot run JetPack 7.2 (no vendor BSP). Then:
  JetPack 6.2 + Isaac ROS 3.2 + `ROS_DISTRO=humble`, and start planning the jump straight to Lyrical.
- ArduPilot and ardupilot_gazebo officially support Lyrical and Isaac ROS 5.x is stable: migrate.
- In any case, before **May 2029** (Jazzy end of life).
