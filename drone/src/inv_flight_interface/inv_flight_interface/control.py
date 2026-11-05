"""Pure math for the flight interface (no ROS imports), so it can be unit tested in isolation."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

Vec3 = tuple[float, float, float]


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return (c * x - s * y, s * x + c * y)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class Limits:
    max_horizontal_speed: float = 1.0
    max_vertical_speed: float = 0.5
    max_yaw_rate: float = 0.8
    max_accel: float = 1.5


@dataclass(frozen=True)
class Gains:
    xy: float = 0.8
    z: float = 1.0
    yaw: float = 1.2


@dataclass(frozen=True)
class Box:
    lo: Vec3
    hi: Vec3

    def contains(self, p: Vec3, margin: float = 0.0) -> bool:
        return all(self.lo[i] + margin <= p[i] <= self.hi[i] - margin for i in range(3))

    def limit_velocity(self, p: Vec3, v: Vec3, margin: float) -> Vec3:
        """Zero every velocity component that pushes the vehicle into a wall it is within `margin` of."""
        out = list(v)
        for i in range(3):
            if p[i] <= self.lo[i] + margin and out[i] < 0.0:
                out[i] = 0.0
            if p[i] >= self.hi[i] - margin and out[i] > 0.0:
                out[i] = 0.0
        return (out[0], out[1], out[2])


@dataclass(frozen=True)
class Setpoint:
    """Velocity setpoint in the map frame (ENU) plus yaw rate (CCW positive)."""

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0

    @property
    def horizontal_speed(self) -> float:
        return math.hypot(self.vx, self.vy)


def goto_setpoint(
    position: Vec3,
    yaw: float,
    target: Vec3,
    target_yaw: float,
    gains: Gains,
    limits: Limits,
    max_speed: float | None = None,
) -> Setpoint:
    """Proportional position/yaw controller with speed limits (the FC does the inner loops)."""
    ex, ey, ez = (target[i] - position[i] for i in range(3))
    vx, vy = gains.xy * ex, gains.xy * ey
    speed_limit = limits.max_horizontal_speed if not max_speed else min(max_speed, limits.max_horizontal_speed)
    speed = math.hypot(vx, vy)
    if speed > speed_limit > 0.0:
        vx, vy = vx * speed_limit / speed, vy * speed_limit / speed
    vz = clamp(gains.z * ez, limits.max_vertical_speed)
    yaw_rate = clamp(gains.yaw * wrap_angle(target_yaw - yaw), limits.max_yaw_rate)
    return Setpoint(vx, vy, vz, yaw_rate)


def slew(previous: Setpoint, wanted: Setpoint, max_accel: float, dt: float) -> Setpoint:
    """Limit the change in horizontal/vertical velocity to max_accel * dt."""
    if dt <= 0.0 or max_accel <= 0.0:
        return wanted
    step = max_accel * dt
    dvx, dvy = wanted.vx - previous.vx, wanted.vy - previous.vy
    dh = math.hypot(dvx, dvy)
    if dh > step:
        dvx, dvy = dvx * step / dh, dvy * step / dh
    dvz = clamp(wanted.vz - previous.vz, step)
    return Setpoint(previous.vx + dvx, previous.vy + dvy, previous.vz + dvz, wanted.yaw_rate)


def position_error(position: Vec3, target: Vec3) -> float:
    return math.dist(position, target)


class FrameAlignment:
    """Learn the rotation between our odom frame and the flight controller's ENU "map" frame.

    AP_DDS (Copter-4.7.2) treats the odom frame of the external navigation it receives as x-forward,
    y-left, z-up and maps its +x axis to North (AP_DDS_External_Odom::convert_transform). It then
    reports pose and accepts "map"-frame velocity commands in ENU built from that North
    (AP_DDS_Client pose topic, AP_DDS_External_Control::handle_velocity_control). The net effect
    is that the FC's map frame is our odom frame rotated by +90 deg about z.

    Rather than hard-code that, we measure it: yaw reported by the FC minus the yaw we fed it,
    averaged over a window and snapped to the nearest quarter turn. If ArduPilot ever changes the
    convention, this picks up the new rotation instead of flying sideways.
    """

    def __init__(
        self,
        window_s: float = 2.0,
        max_spread_rad: float = math.radians(5.0),
        snap_tolerance_rad: float = math.radians(10.0),
    ):
        self._window_s = window_s
        self._max_spread = max_spread_rad
        self._snap_tolerance = snap_tolerance_rad
        self._samples: deque[tuple[float, float]] = deque()
        self.rotation: float | None = None
        self.residual: float | None = None

    def update(self, t: float, fc_yaw: float, odom_yaw: float) -> None:
        self._samples.append((t, wrap_angle(fc_yaw - odom_yaw)))
        while self._samples and t - self._samples[0][0] > self._window_s:
            self._samples.popleft()
        if len(self._samples) < 5 or t - self._samples[0][0] < 0.8 * self._window_s:
            return
        ref = self._samples[0][1]
        deltas = [wrap_angle(d - ref) for _, d in self._samples]
        if max(deltas) - min(deltas) > self._max_spread:
            return
        mean = wrap_angle(ref + sum(deltas) / len(deltas))
        snapped = round(mean / (math.pi / 2)) * (math.pi / 2)
        self.residual = wrap_angle(mean - snapped)
        if abs(self.residual) <= self._snap_tolerance:
            self.rotation = wrap_angle(snapped)

    @property
    def ready(self) -> bool:
        return self.rotation is not None

    def reset(self) -> None:
        self._samples.clear()
        self.rotation = None
        self.residual = None
