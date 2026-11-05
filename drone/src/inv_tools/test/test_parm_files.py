import pathlib

import pytest

from inv_tools.parm_files import diff, load_layers, parse_parm_text
from inv_tools.record import build_command

ROOT = pathlib.Path(__file__).resolve().parents[4]
COMMON = ROOT / "real" / "flight_controller" / "params" / "common.parm"
SIM = ROOT / "sim" / "params" / "sim.parm"
REAL = ROOT / "real" / "flight_controller" / "params" / "real.parm"


def test_parse_formats():
    params = parse_parm_text("A_B 1\nC,2.5  # comment\n# only a comment\n\nD\t-3e-2\n")
    assert params == {"A_B": 1.0, "C": 2.5, "D": -0.03}


def test_parse_rejects_garbage():
    with pytest.raises(ValueError, match="cannot parse"):
        parse_parm_text("NOT A PARAM LINE\n")


@pytest.mark.skipif(not COMMON.exists(), reason="repo param files not available")
def test_committed_param_files_parse_and_layer():
    sim = load_layers([COMMON, SIM])
    real = load_layers([COMMON, REAL])
    # The GPS-denied EKF setup is shared, so sim exercises the same path as the vehicle.
    for name in ("EK3_SRC1_POSXY", "EK3_SRC1_YAW", "VISO_TYPE", "GPS1_TYPE", "DDS_ENABLE"):
        assert sim[name][0] == real[name][0], name
    assert sim["EK3_SRC1_POSXY"][0] == 6  # ExternalNav
    assert sim["SIM_GPS1_ENABLE"] == (0.0, "sim.parm")
    assert "SIM_GPS1_ENABLE" not in real


def test_diff():
    expected = {"A": (1.0, "x.parm"), "B": (2.0, "x.parm"), "C": (3.0, "y.parm")}
    mismatches, unknown = diff(expected, {"A": 1.0000001, "B": 2.5})
    assert mismatches == [("B", 2.0, 2.5, "x.parm")]
    assert unknown == [("C", 3.0, "y.parm")]


def test_record_command():
    cmd = build_command(["/tf", "/flight/state"], "bags/x", 10)
    assert cmd[:4] == ["ros2", "bag", "record", "--storage"]
    assert "mcap" in cmd and cmd[-2:] == ["/tf", "/flight/state"]
