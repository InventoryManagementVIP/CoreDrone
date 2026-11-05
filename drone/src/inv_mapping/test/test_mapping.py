import pathlib

import yaml

from inv_mapping.corridor_checker import DEFAULTS


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "corridor_checker.yaml"
    assert yaml.safe_load(path.read_text())["corridor_checker"]["ros__parameters"] == DEFAULTS
