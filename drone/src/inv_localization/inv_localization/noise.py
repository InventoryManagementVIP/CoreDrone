"""Noise and drift model for the ground-truth backend, so sim runs can mimic VSLAM error.

All randomness comes from one seeded generator, so a run is reproducible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class NoiseConfig:
    position_noise_m: float = 0.0  # white noise, std dev per axis
    yaw_noise_rad: float = 0.0
    drift_m_per_sqrt_s: float = 0.0  # horizontal random walk, like VO drift
    yaw_drift_rad_per_sqrt_s: float = 0.0
    seed: int = 0

    @property
    def enabled(self) -> bool:
        return any(
            v > 0.0
            for v in (self.position_noise_m, self.yaw_noise_rad, self.drift_m_per_sqrt_s, self.yaw_drift_rad_per_sqrt_s)
        )


class NoiseModel:
    def __init__(self, config: NoiseConfig):
        self.config = config
        self._rng = np.random.default_rng(config.seed)
        self._drift_xy = np.zeros(2)
        self._drift_yaw = 0.0
        self._last_t: float | None = None

    def sample(self, t: float) -> tuple[np.ndarray, float]:
        """Return (xyz error, yaw error) at time t (seconds)."""
        c = self.config
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        if dt > 0.0:
            self._drift_xy += self._rng.normal(0.0, c.drift_m_per_sqrt_s * math.sqrt(dt), 2)
            self._drift_yaw += float(self._rng.normal(0.0, c.yaw_drift_rad_per_sqrt_s * math.sqrt(dt)))
        err = np.zeros(3)
        err[:2] = self._drift_xy
        if c.position_noise_m > 0.0:
            err += self._rng.normal(0.0, c.position_noise_m, 3)
        yaw = self._drift_yaw + (float(self._rng.normal(0.0, c.yaw_noise_rad)) if c.yaw_noise_rad > 0.0 else 0.0)
        return err, yaw


def parse_dropouts(spec: str) -> list[tuple[float, float]]:
    """'20:3,60:5' -> [(20.0, 23.0), (60.0, 65.0)]: windows (sim seconds) with no localization."""
    windows = []
    for part in filter(None, (p.strip() for p in spec.split(","))):
        start, duration = (float(v) for v in part.split(":"))
        windows.append((start, start + duration))
    return windows


def in_dropout(t: float, windows: list[tuple[float, float]]) -> bool:
    return any(a <= t < b for a, b in windows)
