# sim/: simulation platform

Platform-specific things only: worlds, models, the platform launch, SITL parameter overrides,
evaluation and simulation tests. All autonomy code lives in `drone/src` and runs unchanged here.
Full guide: [docs/simulation.md](../docs/simulation.md).

| Path | What |
|---|---|
| `worldgen/` | layout YAML → world SDF, label textures, drone model, `ground_truth.json` (`just worldgen`) |
| `models/inv_quad/sensors.yaml` | simulated sensor intrinsics and rates (poses come from the URDF) |
| `worlds/generated/` | generated worlds (gitignored); hand-made worlds go next to it in `worlds/` |
| `launch/sim_platform.launch.py` | Gazebo + SITL + micro-ROS agent + bridges + `fc_bootstrap` |
| `params/sim.parm` | SITL overrides on top of `real/flight_controller/params/common.parm` |
| `evaluation/` | score a run against ground truth (`python3 -m evaluation`) |
| `tests/` | `test_sim_smoke.py` (CI), `test_sim_mission.py` (nightly), `check_param_names.sh` |
| `isaac/` | placeholder: when and how to add Isaac Sim |
