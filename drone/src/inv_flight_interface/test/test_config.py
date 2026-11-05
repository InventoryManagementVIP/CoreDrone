"""config/flight_interface.yaml and the node's declared defaults must not drift apart."""

import pathlib

import yaml

from inv_flight_interface.copter_modes import mode_id, mode_name
from inv_flight_interface.node import DEFAULTS


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "flight_interface.yaml"
    params = yaml.safe_load(path.read_text())["flight_interface"]["ros__parameters"]
    assert params == DEFAULTS


def test_copter_modes():
    assert mode_id("guided") == 4
    assert mode_id("LAND") == 9
    assert mode_name(5) == "LOITER"
    assert mode_name(None) == ""
