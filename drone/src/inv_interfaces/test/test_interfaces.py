"""The generated interfaces import, and their string constants match the backend contract."""

import json
import pathlib

import pytest


def test_messages_import():
    from inv_interfaces.msg import (  # noqa: F401
        CorridorStatus,
        FlightState,
        InventoryItem,
        InventoryItemArray,
        LabelDetection,
        LabelDetectionArray,
        LocalizationStatus,
        MissionStatus,
        SafetyStatus,
    )


def test_services_and_actions_import():
    from inv_interfaces.action import GoTo, Hover, Land, Takeoff  # noqa: F401
    from inv_interfaces.srv import Abort, ExportCsv  # noqa: F401


def _contract_path() -> pathlib.Path:
    # drone/src/inv_interfaces/test -> repo root
    return pathlib.Path(__file__).resolve().parents[4] / "contracts" / "scan_record.schema.json"


def test_symbologies_match_contract():
    from inv_interfaces.msg import LabelDetection

    path = _contract_path()
    if not path.exists():
        pytest.skip(f"contract not found at {path} (package built outside the repo)")
    schema = json.loads(path.read_text())
    allowed = set(schema["properties"]["symbology"]["enum"])
    ours = {
        LabelDetection.SYMBOLOGY_QR,
        LabelDetection.SYMBOLOGY_EAN13,
        LabelDetection.SYMBOLOGY_CODE128,
        LabelDetection.SYMBOLOGY_APRILTAG,
        LabelDetection.SYMBOLOGY_OTHER,
    }
    assert ours == allowed


def test_attribution_methods_match_contract():
    from inv_interfaces.msg import InventoryItem

    path = _contract_path()
    if not path.exists():
        pytest.skip(f"contract not found at {path}")
    schema = json.loads(path.read_text())
    allowed = set(schema["properties"]["attribution_method"]["enum"])
    ours = {
        InventoryItem.ATTRIBUTION_LABEL,
        InventoryItem.ATTRIBUTION_POSE,
        InventoryItem.ATTRIBUTION_NONE,
    }
    assert ours == allowed
