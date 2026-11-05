import pytest

from inv_flight_interface.time_sync import ClockOffsetEstimator


def test_offset_is_the_least_delayed_sample():
    est = ClockOffsetEstimator(window_s=5.0)
    true_offset = -1000.0  # FC booted 1000 s after the ROS clock's zero
    for i, latency in enumerate([0.004, 0.001, 0.010, 0.002]):
        ros_t = 2000.0 + i * 0.01
        est.update(fc_time_s=ros_t - latency + true_offset, ros_time_s=ros_t)
    assert est.offset_s == pytest.approx(true_offset - 0.001)
    assert est.to_fc(2100.0) == pytest.approx(1100.0 - 0.001)


def test_old_samples_expire():
    est = ClockOffsetEstimator(window_s=1.0)
    est.update(fc_time_s=10.0, ros_time_s=10.0)  # zero latency, offset 0
    est.update(fc_time_s=11.99, ros_time_s=12.0)  # 10 ms latency, 2 s later
    assert est.offset_s == pytest.approx(-0.01)


def test_clock_reset_starts_over():
    est = ClockOffsetEstimator(max_jump_s=0.5)
    est.update(fc_time_s=100.0, ros_time_s=50.0)
    est.update(fc_time_s=1.0, ros_time_s=51.0)  # SITL restarted
    assert est.offset_s == pytest.approx(-50.0)


def test_utc_fc_clock_is_flagged():
    est = ClockOffsetEstimator()
    est.update(fc_time_s=1.79e9, ros_time_s=10.0)
    assert est.fc_clock_is_utc
    est.update(fc_time_s=120.0, ros_time_s=10.0)
    assert not est.fc_clock_is_utc


def test_no_offset_before_first_sample():
    assert ClockOffsetEstimator().to_fc(1.0) is None
