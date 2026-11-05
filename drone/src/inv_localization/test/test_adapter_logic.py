import math
import pathlib

import yaml

from inv_interfaces.msg import LocalizationStatus
from inv_localization import adapter, groundtruth
from inv_localization.adapter import classify

TRACKING, DEGRADED, LOST = (
    LocalizationStatus.STATE_TRACKING,
    LocalizationStatus.STATE_DEGRADED,
    LocalizationStatus.STATE_LOST,
)


def test_classify():
    assert classify(0.05, 30.0, None, 0.5, 15.0) == TRACKING
    assert classify(0.05, 10.0, None, 0.5, 15.0) == DEGRADED
    assert classify(0.80, 30.0, None, 0.5, 15.0) == LOST
    assert classify(math.inf, 0.0, None, 0.5, 15.0) == LOST
    assert classify(0.05, 30.0, LOST, 0.5, 15.0) == LOST  # backend says it lost tracking
    assert classify(0.05, 30.0, DEGRADED, 0.5, 15.0) == DEGRADED


def _params(name, node):
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / name
    return yaml.safe_load(path.read_text())[node]["ros__parameters"]


def test_configs_match_defaults():
    assert _params("adapter.yaml", "localization_adapter") == adapter.DEFAULTS
    assert _params("groundtruth.yaml", "localization_groundtruth") == groundtruth.DEFAULTS
