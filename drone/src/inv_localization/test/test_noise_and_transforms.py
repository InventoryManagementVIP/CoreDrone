import math

import numpy as np
import pytest

from inv_localization.noise import NoiseConfig, NoiseModel, in_dropout, parse_dropouts
from inv_localization.transforms import matrix_to_quat, quat_to_matrix, rot_z, yaw_of_matrix


def test_disabled_noise_is_exact():
    model = NoiseModel(NoiseConfig())
    for t in np.arange(0.0, 10.0, 0.1):
        err, yaw = model.sample(float(t))
        assert np.allclose(err, 0.0)
        assert yaw == 0.0


def test_noise_is_reproducible_with_a_seed():
    cfg = NoiseConfig(position_noise_m=0.02, drift_m_per_sqrt_s=0.01, yaw_noise_rad=0.01, seed=7)
    a, b = NoiseModel(cfg), NoiseModel(cfg)
    for t in np.arange(0.0, 5.0, 0.02):
        ea, ya = a.sample(float(t))
        eb, yb = b.sample(float(t))
        assert np.allclose(ea, eb) and ya == yb


def test_drift_grows_like_a_random_walk():
    # Std dev after T seconds is rate * sqrt(T); check the average over many seeds.
    finals = []
    for seed in range(150):
        model = NoiseModel(NoiseConfig(drift_m_per_sqrt_s=0.01, seed=seed))
        for t in np.arange(0.0, 100.0, 2.0):
            err, _ = model.sample(float(t))
        finals.append(err[0])
    # The last sample is at t = 98 s.
    assert np.std(finals) == pytest.approx(0.01 * math.sqrt(98.0), rel=0.3)


def test_dropouts():
    windows = parse_dropouts("20:3, 60:5")
    assert windows == [(20.0, 23.0), (60.0, 65.0)]
    assert in_dropout(21.0, windows) and not in_dropout(23.0, windows)
    assert parse_dropouts("") == []


def test_quaternion_round_trip():
    for yaw in (0.0, 0.5, -2.0, math.pi - 1e-6):
        q = matrix_to_quat(rot_z(yaw)[:3, :3])
        assert yaw_of_matrix(np.pad(quat_to_matrix(*q), ((0, 1), (0, 1)))) == pytest.approx(yaw)
