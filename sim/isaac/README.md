# Isaac Sim (placeholder)

**Not in scope now** (docs/adr/0005-gazebo-harmonic.md). Gazebo Harmonic + ArduPilot SITL is the
simulator: it runs without a GPU, headless in CI, and is ArduPilot's supported path.

## When to add Isaac Sim

- Label decoding has to be validated under realistic lighting, materials, motion blur and distance
  (Gazebo's rendering is too clean to predict real read rates).
- VSLAM (cuVSLAM) needs photoreal, textured scenes to be tested meaningfully in simulation.
- Someone on the team has an RTX GPU to run it (Isaac Sim does not run on Macs or CI runners).

## How it should fit

- A second sim platform: `sim/isaac/launch/isaac_platform.launch.py`, publishing the same canonical
  topics (docs/interfaces.md) and talking to ArduPilot SITL the same way (JSON physics bridge).
  The autonomy stack must not change.
- Generate the USD stage from the same layout file (`config/warehouses/*.yaml`) and the same
  ground truth, so `sim/evaluation` scores both simulators identically.
- Drone sensor poses from the URDF (`inv_description`), never copied by hand.
