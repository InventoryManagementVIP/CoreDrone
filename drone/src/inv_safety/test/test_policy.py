import math
import pathlib

import yaml

from inv_safety.monitor import DEFAULTS
from inv_safety.policy import (
    ABORT,
    OK,
    WARN,
    Inputs,
    SafetyConfig,
    evaluate,
    should_request_abort,
    should_resend_abort,
)

CFG = SafetyConfig()


def flying(**kw) -> Inputs:
    base = {
        "now_s": 100.0,
        "flight_state_age_s": 0.1,
        "connected": True,
        "armed": True,
        "flying": True,
        "battery_fraction": 0.8,
    }
    base.update(kw)
    return Inputs(**base)


def test_all_good():
    assert evaluate(CFG, flying()).level == OK


def test_low_battery_warns_then_aborts():
    assert evaluate(CFG, flying(battery_fraction=0.25)).level == WARN
    v = evaluate(CFG, flying(battery_fraction=0.15))
    assert v.level == ABORT and "battery" in v.reasons[0]


def test_unknown_battery_warns_only_while_flying():
    v = evaluate(CFG, flying(battery_fraction=math.nan))
    assert v.level == WARN and "battery" in v.reasons[0]
    assert evaluate(CFG, flying(battery_fraction=math.nan, flying=False)).level == OK


def test_abort_is_resent_until_seen_landing():
    assert should_resend_abort(CFG, flying(), False, 1.0)
    assert not should_resend_abort(CFG, flying(), False, 0.5)  # retry period
    assert not should_resend_abort(CFG, flying(), True, 1.0)  # the flight interface is acting on it
    assert not should_resend_abort(CFG, flying(armed=False), False, 1.0)
    assert not should_resend_abort(CFG, flying(pilot_override=True), False, 1.0)  # the pilot always wins


def test_localization_lost_needs_grace_period():
    assert evaluate(CFG, flying(localization_lost_since_s=99.5)).level == OK
    assert evaluate(CFG, flying(localization_lost_since_s=98.0)).level == ABORT


def test_localization_lost_on_the_ground_only_warns():
    assert evaluate(CFG, flying(flying=False, armed=False, localization_lost_since_s=50.0)).level == WARN


def test_stale_flight_state_aborts_in_flight():
    assert evaluate(CFG, flying(flight_state_age_s=5.0)).level == ABORT


def test_blocked_corridor_only_warns():
    assert evaluate(CFG, flying(corridor_blocked=True)).level == WARN


def test_pilot_override_always_wins():
    x = flying(battery_fraction=0.05, pilot_override=True)
    assert evaluate(CFG, x).level == ABORT
    assert not should_request_abort(evaluate(CFG, x), x, already_requested=False)


def test_abort_requested_once():
    x = flying(battery_fraction=0.05)
    v = evaluate(CFG, x)
    assert should_request_abort(v, x, already_requested=False)
    assert not should_request_abort(v, x, already_requested=True)


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "safety.yaml"
    assert yaml.safe_load(path.read_text())["safety_monitor"]["ros__parameters"] == DEFAULTS
