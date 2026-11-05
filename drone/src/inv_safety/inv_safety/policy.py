"""Safety policy: health inputs -> OK / WARN / ABORT. Pure Python so it can be unit tested.

Phase 1 policy (docs/architecture.md, "Safety"):
  * battery below the abort fraction                      -> ABORT
  * localization LOST for longer than the grace period    -> ABORT
  * flight-controller link lost                            -> ABORT (the FC's own failsafes act too)
  * flight state stale (flight interface not running)      -> ABORT while flying
  * corridor blocked                                       -> WARN (the mission holds; no abort)
The RC safety pilot always wins: during pilot override nothing is requested.
TODO(phase-2): smarter policies (battery vs distance home, repeated degraded tracking, geofence margin).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

OK, WARN, ABORT = 0, 1, 2


@dataclass(frozen=True)
class SafetyConfig:
    battery_warn_fraction: float = 0.30
    battery_abort_fraction: float = 0.20
    localization_lost_grace_s: float = 1.0
    fc_link_grace_s: float = 2.0
    state_stale_s: float = 2.0


@dataclass
class Inputs:
    now_s: float
    flight_state_age_s: float = math.inf
    connected: bool = False
    armed: bool = False
    flying: bool = False
    pilot_override: bool = False
    battery_fraction: float = math.nan
    localization_lost_since_s: float | None = None
    localization_degraded: bool = False
    corridor_blocked: bool = False
    fc_link_lost_since_s: float | None = None


@dataclass
class Verdict:
    level: int = OK
    reasons: list[str] = field(default_factory=list)

    def raise_to(self, level: int, reason: str) -> None:
        self.level = max(self.level, level)
        self.reasons.append(reason)


def evaluate(cfg: SafetyConfig, x: Inputs) -> Verdict:
    v = Verdict()
    if x.flight_state_age_s > cfg.state_stale_s:
        v.raise_to(ABORT if x.flying else WARN, f"no flight state for {x.flight_state_age_s:.1f} s")
        return v
    if x.fc_link_lost_since_s is not None and x.now_s - x.fc_link_lost_since_s > cfg.fc_link_grace_s:
        v.raise_to(ABORT if x.armed else WARN, "flight controller link lost")
    if not math.isnan(x.battery_fraction):
        if x.battery_fraction < cfg.battery_abort_fraction:
            v.raise_to(ABORT, f"battery {x.battery_fraction:.0%} < {cfg.battery_abort_fraction:.0%}")
        elif x.battery_fraction < cfg.battery_warn_fraction:
            v.raise_to(WARN, f"battery {x.battery_fraction:.0%} < {cfg.battery_warn_fraction:.0%}")
    if x.localization_lost_since_s is not None:
        lost_for = x.now_s - x.localization_lost_since_s
        if lost_for > cfg.localization_lost_grace_s:
            v.raise_to(ABORT if x.flying else WARN, f"localization lost for {lost_for:.1f} s")
    elif x.localization_degraded:
        v.raise_to(WARN, "localization degraded")
    if x.corridor_blocked:
        v.raise_to(WARN, "corridor ahead blocked")
    return v


def should_request_abort(verdict: Verdict, x: Inputs, already_requested: bool) -> bool:
    return verdict.level == ABORT and x.armed and x.flying and not x.pilot_override and not already_requested
