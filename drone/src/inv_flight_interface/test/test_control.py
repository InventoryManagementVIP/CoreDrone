import math

import pytest

from inv_flight_interface.control import (
    Box,
    FrameAlignment,
    Gains,
    Limits,
    Setpoint,
    goto_setpoint,
    rotate_xy,
    slew,
    wrap_angle,
    yaw_from_quaternion,
)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_yaw_from_quaternion():
    half = math.radians(30) / 2
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(math.radians(30))


def test_goto_speed_is_limited_and_points_at_target():
    limits = Limits(max_horizontal_speed=1.0, max_vertical_speed=0.5, max_yaw_rate=0.8)
    sp = goto_setpoint((0, 0, 1), 0.0, (10, 0, 1), 0.0, Gains(), limits)
    assert sp.vx == pytest.approx(1.0)
    assert sp.vy == pytest.approx(0.0)
    assert sp.vz == pytest.approx(0.0)


def test_goto_per_goal_speed_cap():
    sp = goto_setpoint((0, 0, 1), 0.0, (0, 10, 1), 0.0, Gains(), Limits(max_horizontal_speed=1.0), max_speed=0.3)
    assert sp.horizontal_speed == pytest.approx(0.3)


def test_goto_vertical_and_yaw_limits():
    limits = Limits(max_vertical_speed=0.5, max_yaw_rate=0.8)
    sp = goto_setpoint((0, 0, 0), 0.0, (0, 0, 5), math.pi, Gains(), limits)
    assert sp.vz == pytest.approx(0.5)
    assert abs(sp.yaw_rate) == pytest.approx(0.8)


def test_goto_proportional_near_target():
    sp = goto_setpoint((0, 0, 1), 0.0, (0.1, 0, 1), 0.0, Gains(xy=0.8), Limits())
    assert sp.vx == pytest.approx(0.08)


def test_slew_limits_acceleration():
    out = slew(Setpoint(), Setpoint(vx=1.0), max_accel=1.0, dt=0.1)
    assert out.vx == pytest.approx(0.1)
    out = slew(Setpoint(), Setpoint(vz=-1.0), max_accel=1.0, dt=0.1)
    assert out.vz == pytest.approx(-0.1)


def test_box_blocks_outward_velocity_only():
    box = Box((0, 0, 0), (10, 10, 3))
    assert box.limit_velocity((0.1, 5, 1), (-1.0, 0.5, 0.0), margin=0.3) == (0.0, 0.5, 0.0)
    assert box.limit_velocity((0.1, 5, 1), (1.0, 0.5, 0.0), margin=0.3) == (1.0, 0.5, 0.0)
    assert box.limit_velocity((5, 5, 2.9), (0, 0, 0.5), margin=0.3) == (0.0, 0.0, 0.0)
    assert box.contains((5, 5, 1), margin=0.3)
    assert not box.contains((5, 5, 2.9), margin=0.3)


def test_rotate_xy():
    x, y = rotate_xy(1.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 1.0))


def _feed(alignment, delta, start=0.0, seconds=3.0, odom_yaw=0.3):
    t = start
    while t < start + seconds:
        alignment.update(t, odom_yaw + delta, odom_yaw)
        t += 0.05


def test_frame_alignment_learns_ardupilot_convention():
    # Copter-4.7.2: FC map frame = odom rotated +90 deg.
    alignment = FrameAlignment()
    _feed(alignment, math.radians(90.0) + math.radians(1.0))
    assert alignment.ready
    assert alignment.rotation == pytest.approx(math.pi / 2)
    assert math.degrees(alignment.residual) == pytest.approx(1.0, abs=0.01)


def test_frame_alignment_follows_a_future_convention_change():
    alignment = FrameAlignment()
    _feed(alignment, math.radians(90.0))
    _feed(alignment, 0.0, start=10.0)
    assert alignment.rotation == pytest.approx(0.0)


def test_frame_alignment_waits_for_a_steady_signal():
    alignment = FrameAlignment()
    t = 0.0
    while t < 3.0:  # FC yaw still wandering (EKF not converged)
        alignment.update(t, 0.3 + math.radians(40) * math.sin(t * 5), 0.3)
        t += 0.05
    assert not alignment.ready


def test_frame_alignment_rejects_off_grid_rotation():
    alignment = FrameAlignment()
    _feed(alignment, math.radians(45.0))
    assert not alignment.ready
