"""The parameters drone.launch.py really hands each node, merged the way ROS merges them: in order,
later files winning. Regression: package defaults used to load last and undo the platform's limits.
"""

import importlib.util
import pathlib

import pytest
import yaml
from launch import LaunchContext

ROOT = pathlib.Path(__file__).resolve().parents[4]
LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "drone.launch.py"


def launch_nodes(monkeypatch, tmp_path, **args):
    monkeypatch.setenv("INV_ROOT", str(ROOT))
    spec = importlib.util.spec_from_file_location("drone_launch_under_test", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = {}
    monkeypatch.setattr(module, "Node", lambda **kw: nodes.setdefault(kw["name"], kw))
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "platform": "sim",
            "localization": "",
            "mapping": "none",
            "perception": "none",
            "mission": "scan",
            "mission_id": "",
            "warehouse": "sim_small",
            "with_platform": "false",
            "run_dir": str(tmp_path),
            "foxglove": "true",
            "params_file": "",
            **args,
        }
    )
    module._launch(context)
    return nodes


def effective(node: dict) -> dict:
    merged = {}
    for layer in node["parameters"]:
        if isinstance(layer, dict):
            merged.update(layer)
        else:
            content = yaml.safe_load(pathlib.Path(layer).read_text()) or {}
            merged.update(content.get(node["name"], {}).get("ros__parameters", {}))
    return merged


def test_real_platform_limits_beat_package_defaults(monkeypatch, tmp_path):
    p = effective(launch_nodes(monkeypatch, tmp_path, platform="real")["flight_interface"])
    assert (p["max_horizontal_speed_mps"], p["max_vertical_speed_mps"]) == (0.5, 0.3)


def test_sim_keeps_package_defaults(monkeypatch, tmp_path):
    p = effective(launch_nodes(monkeypatch, tmp_path, platform="sim")["flight_interface"])
    assert (p["max_horizontal_speed_mps"], p["max_vertical_speed_mps"]) == (1.0, 0.5)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_platform_overrides_reach_every_node(monkeypatch, tmp_path, platform):
    nodes = launch_nodes(monkeypatch, tmp_path, platform=platform)
    overrides = yaml.safe_load((LAUNCH_FILE.parents[1] / "config" / f"{platform}.yaml").read_text())
    for name, section in overrides.items():
        applied = effective(nodes[name])
        for key, value in section["ros__parameters"].items():
            assert applied[key] == value, f"{name}.{key}"


def test_user_params_file_wins_over_everything(monkeypatch, tmp_path):
    user = tmp_path / "override.yaml"
    user.write_text(
        "flight_interface:\n  ros__parameters:\n    max_horizontal_speed_mps: 0.2\n    geofence_margin_m: 0.9\n"
    )
    nodes = launch_nodes(monkeypatch, tmp_path, platform="real", params_file=str(user))
    p = effective(nodes["flight_interface"])
    assert (p["max_horizontal_speed_mps"], p["geofence_margin_m"]) == (0.2, 0.9)
    assert p["max_vertical_speed_mps"] == 0.3


def test_computed_values_beat_platform_and_package_files(monkeypatch, tmp_path):
    p = effective(launch_nodes(monkeypatch, tmp_path)["flight_interface"])
    assert p["geofence_max_xyz"] != [50.0, 50.0, 5.0]  # from the warehouse layout
    assert p["mission_heartbeat_timeout_s"] == 5.0
    assert p["safety_heartbeat_timeout_s"] > 0.0


def test_foxglove_stays_off_the_vehicle_network(monkeypatch, tmp_path):
    nodes = launch_nodes(monkeypatch, tmp_path, platform="real")
    assert nodes["foxglove_bridge"]["parameters"][0]["address"] == "127.0.0.1"
