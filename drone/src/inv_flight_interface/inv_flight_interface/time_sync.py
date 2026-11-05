"""Map ROS timestamps into the flight controller's clock.

Why this exists (docs/ardupilot.md, "External navigation timestamps"): at Copter-4.7.2, AP_DDS
hands the header stamp of each /ap/tf transform straight to the EKF as a time in the flight
controller's own boot clock (AP_DDS_External_Odom.cpp: `time_ms = remote_time_us * 1E-3`, with a
TODO for jitter correction). ROS stamps are in ROS time (wall clock on the Jetson, /clock in sim),
so unconverted stamps land seconds or days away from the EKF's time horizon and are never fused.

The flight controller publishes its clock on /ap/time. With no GPS and the default BRD_RTC_TYPES
(GPS only), that clock is the boot clock (AP_HAL::micros64), which is exactly what the EKF needs.

Each /ap/time sample gives offset_meas = fc_time - ros_receive_time = true_offset - latency.
Latency is never negative, so the largest recent measurement is the best estimate of the offset.
"""

from __future__ import annotations

from collections import deque

# A boot clock above this many seconds (~31 years) means the flight controller's RTC was set from
# GPS or MAVLink SYSTEM_TIME, so /ap/time is UTC and cannot be used for EKF timestamps.
UTC_THRESHOLD_S = 1.0e9


class ClockOffsetEstimator:
    def __init__(self, window_s: float = 5.0, max_jump_s: float = 0.5):
        self._window_s = window_s
        self._max_jump_s = max_jump_s
        self._samples: deque[tuple[float, float]] = deque()  # (ros_time, offset_meas)
        self._offset: float | None = None
        self.fc_clock_is_utc = False

    @property
    def offset_s(self) -> float | None:
        """fc_time - ros_time, or None before the first sample."""
        return self._offset

    def update(self, fc_time_s: float, ros_time_s: float) -> None:
        self.fc_clock_is_utc = fc_time_s > UTC_THRESHOLD_S
        offset = fc_time_s - ros_time_s
        # A large jump means one of the clocks was reset (SITL restart, sim reset): start over.
        if self._offset is not None and abs(offset - self._offset) > self._max_jump_s:
            self._samples.clear()
        self._samples.append((ros_time_s, offset))
        while self._samples and ros_time_s - self._samples[0][0] > self._window_s:
            self._samples.popleft()
        self._offset = max(o for _, o in self._samples)

    def to_fc(self, ros_time_s: float) -> float | None:
        if self._offset is None:
            return None
        return ros_time_s + self._offset

    def reset(self) -> None:
        self._samples.clear()
        self._offset = None
