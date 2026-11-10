"""The real command state machine, driven tick by tick against a fake flight controller.

No ROS graph and no Node.__init__: time, pose and the FC are all set by the test, so failures
(rejected requests, stale data, pilot takeover) can be injected exactly.
"""

import math
import threading
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.action import GoalResponse

from inv_flight_interface.backend import FcState
from inv_flight_interface.control import Box, FrameAlignment, Gains, Limits, Setpoint
from inv_flight_interface.node import DEFAULTS, GUIDED, LAND, Command, FlightInterfaceNode, Pose
from inv_interfaces.action import GoTo, Hover
from inv_interfaces.msg import MissionStatus
from inv_interfaces.srv import Abort

LOITER = 5


class FakeBackend:
    def __init__(self, harness):
        self.harness = harness
        self.fc = FcState(armed=True, flying=True, mode_id=GUIDED, battery_percentage=0.8, battery_voltage=15.0)
        self.sent = []
        self.mode_requests = []

    def state(self):
        return self.fc

    def services_ready(self):
        return True

    def request_mode(self, mode_id):
        self.mode_requests.append(mode_id)
        return NS(done=lambda: True, succeeded=lambda: True)

    def send_velocity(self, vx, vy, vz, yaw_rate):
        self.sent.append((vx, vy, vz, yaw_rate))


class Harness(FlightInterfaceNode):
    def __init__(self, **params):  # no super().__init__(): nothing here touches the ROS graph
        self.p = {**DEFAULTS, **params}
        self._lock = threading.RLock()
        self.backend = FakeBackend(self)
        self.alignment = FrameAlignment()
        self.alignment.rotation = 0.0
        self.limits = Limits()
        self.gains = Gains()
        self.fence = Box((-10.0, -10.0, -1.0), (10.0, 10.0, 5.0))
        self.ceiling = Box((-math.inf, -math.inf, -math.inf), (math.inf, math.inf, self.p["max_altitude_m"]))
        self._init_state()
        self.t = 100.0
        self.pose_now = Pose(0.0, 0.0, 1.0, 0.0)
        self.published = []
        self._in_control = True
        self.step(0.0)

    def get_logger(self):
        return NS(info=_ignore, warning=_ignore, error=_ignore)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=Time))

    def _now(self):
        return self.t

    def _pose(self):
        return self.pose_now

    def _map_to_odom_yaw(self):
        return 0.0

    def _publish_state(self, st, connected, now):
        self.published.append(connected)

    def step(self, dt=0.05):
        """Advance the clock with a healthy FC link and external nav, then run one control tick."""
        self.t += dt
        fc = self.backend.fc
        fc.last_message_s = fc.last_status_s = fc.last_battery_s = self.t
        self._last_external_nav_s = self.t
        self._tick()

    def run(self, seconds, dt=0.05):
        for _ in range(round(seconds / dt)):
            self.step(dt)

    @property
    def active(self):
        return self._cmd.kind if self._cmd is not None else None


def _ignore(*_args, **_kwargs):
    pass


def abort(node, reason="test"):
    return node._on_abort(Abort.Request(reason=reason), Abort.Response())


def goto_goal(x=1.0, y=0.0, z=1.0, **fields):
    goal = GoTo.Goal(**fields)
    goal.target.header.frame_id = "map"
    goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = x, y, z
    goal.target.pose.orientation.w = 1.0
    return goal


def test_abort_cannot_be_preempted_then_lands():
    node = Harness()
    assert abort(node).accepted
    assert node.active == "abort"
    # New work is refused at the goal callback and, for a goal already past it, at _start.
    assert node._goal_flying(None) == GoalResponse.REJECT
    assert node._goal_goto(goto_goal()) == GoalResponse.REJECT
    late = Command("hover", {"duration_s": 5.0})
    node._start(late)
    assert late.done.is_set() and not late.success
    assert node.active == "abort"
    node.run(DEFAULTS["abort_hold_s"] + 0.1)
    assert node.active == "land"
    assert node.backend.mode_requests == [LAND]


def test_streaming_velocity_cannot_preempt_recovery():
    node = Harness()
    abort(node)
    msg = TwistStamped()
    msg.header.frame_id = "map"
    msg.twist.linear.x = 1.0
    node._on_cmd_vel(msg)
    assert node.active == "abort"


def test_landing_is_retried_until_the_vehicle_disarms():
    node = Harness()
    node._start(Command("land"))
    node.run(DEFAULTS["land_timeout_s"] + 1.0, dt=0.5)  # FC ignores the mode change the whole time
    assert node.active == "land", "a timed-out landing must start again while still armed"
    assert node.backend.mode_requests.count(LAND) > 1
    assert node._goal_flying(None) == GoalResponse.REJECT
    node.backend.fc.armed = False
    node.step()
    assert node.active is None and not node._recovering
    assert node._goal_takeoff(NS(altitude_m=1.0)) == GoalResponse.ACCEPT  # the latch ends with the flight


def test_pilot_override_beats_recovery():
    node = Harness()
    abort(node)
    node.backend.fc.mode_id = LOITER
    node.step()
    assert node._pilot_override and not node._recovering and node.active is None
    node.backend.sent.clear()
    node.run(1.0)
    assert node.backend.sent == [] and LAND not in node.backend.mode_requests
    assert not abort(node).accepted


@pytest.mark.parametrize("state", [MissionStatus.STATE_FAILED, MissionStatus.STATE_ABORTED])
def test_mission_giving_up_in_the_air_lands(state):
    node = Harness()
    node._hold = Pose(0.0, 0.0, 1.0, 0.0)
    node._on_mission_status(MissionStatus(state=state))
    assert node.active == "land" and node._recovering


def test_running_mission_status_is_only_a_heartbeat():
    node = Harness()
    node._on_mission_status(MissionStatus(state=MissionStatus.STATE_RUNNING))
    assert node.active is None and not node._recovering


def test_mission_giving_up_on_the_ground_does_nothing():
    node = Harness()
    node.backend.fc.flying = False
    node._on_mission_status(MissionStatus(state=MissionStatus.STATE_FAILED))
    assert node.active is None


def test_pilot_keeps_control_when_the_mission_gives_up():
    node = Harness()
    node._pilot_override = True
    node._on_mission_status(MissionStatus(state=MissionStatus.STATE_ABORTED))
    assert node.active is None


def test_safety_monitor_loss_lands():
    node = Harness(safety_heartbeat_timeout_s=3.0)
    node._on_safety_status(None)
    node.run(2.0)
    assert node.active is None
    node.run(1.5)
    assert node.active == "land"


def test_lost_localization_stops_at_once_then_lands():
    node = Harness()
    node._hold = Pose(0.0, 0.0, 1.0, 0.0)
    node.pose_now = None
    node.backend.sent.clear()
    node.run(1.0)
    assert node.backend.sent and set(node.backend.sent) == {(0.0, 0.0, 0.0, 0.0)}
    assert node.active is None
    node.run(DEFAULTS["nav_lost_land_s"])
    assert node.active == "land"


def test_localization_that_returns_cancels_the_landing_countdown():
    node = Harness()
    node.pose_now = None
    node.run(2.0)
    node.pose_now = Pose(0.0, 0.0, 1.0, 0.0)
    node.run(0.2)
    node.pose_now = None
    node.run(2.0)
    assert node.active is None


def test_stale_external_nav_stops_a_goto():
    node = Harness()
    node._start(Command("goto", {"target": (5.0, 0.0, 1.0), "yaw": 0.0, "max_speed": None, "tol": 0.1, "yaw_tol": 0.1}))
    node.step()
    assert node.backend.sent[-1][0] > 0.0
    node._last_external_nav_s = node.t - 1.0
    node.t += 0.05
    node.backend.fc.last_message_s = node.backend.fc.last_status_s = node.t
    node._tick()
    assert node.backend.sent[-1] == (0.0, 0.0, 0.0, 0.0)


def test_takeoff_is_left_to_the_fc_when_localization_drops():
    node = Harness()
    node._start(Command("takeoff", {"altitude_m": 1.0}))
    node._cmd.phase = "climb"
    node.pose_now = None
    node.backend.sent.clear()
    node.step()
    assert node.backend.sent == []


def test_stale_fc_status_means_disconnected():
    node = Harness()
    node.step()
    assert node._connected(node.backend.fc, node.t)
    # /ap/time keeps refreshing last_message_s while /ap/status has stopped.
    node.backend.fc.last_status_s = node.t - DEFAULTS["status_timeout_s"] - 0.1
    assert not node._connected(node.backend.fc, node.t)


def real_pose_lookup(stamp_s, **translation):
    tf = TransformStamped()
    tf.header.stamp.sec = int(stamp_s)
    tf.transform.rotation.w = 1.0
    for axis, value in translation.items():
        setattr(tf.transform.translation, axis, value)
    return NS(p=DEFAULTS, _lookup=lambda *_: tf, _now=lambda: 100.0)


@pytest.mark.parametrize(
    "stamp_s, translation, usable",
    [
        (100.0, {}, True),
        (1.0, {}, False),  # 99 s old
        (200.0, {}, False),  # stamped in the future
        (100.0, {"x": math.nan}, False),
    ],
)
def test_pose_rejects_old_future_and_nonfinite_transforms(stamp_s, translation, usable):
    pose = FlightInterfaceNode._pose(real_pose_lookup(stamp_s, **translation))
    assert (pose is not None) == usable


def test_stale_battery_reads_as_unknown():
    node = Harness()
    published = []
    node._state_pub = NS(publish=published.append)
    st = node.backend.fc
    st.last_battery_s = node.t - DEFAULTS["battery_timeout_s"] - 0.1
    FlightInterfaceNode._publish_state(node, st, True, node.t)
    assert math.isnan(published[-1].battery_percentage) and math.isnan(published[-1].battery_voltage)


def test_slew_cannot_restore_velocity_at_the_fence():
    node = Harness()
    node.fence = Box((-1.0, -1.0, 0.0), (1.0, 1.0, 5.0))
    node._last_setpoint = Setpoint(1.0, 0.0, 0.0)
    node._send(Setpoint(1.0, 0.0, 0.0), Pose(0.7, 0.0, 1.0, 0.0), 0.05)  # on the fence's margin
    assert node.backend.sent[-1][0] == 0.0


def test_slew_cannot_restore_velocity_at_the_ceiling():
    node = Harness()
    node._last_setpoint = Setpoint(0.0, 0.0, 0.5)
    node._send(Setpoint(0.0, 0.0, 0.5), Pose(0.0, 0.0, DEFAULTS["max_altitude_m"], 0.0), 0.05)
    assert node.backend.sent[-1][2] == 0.0


def test_vehicle_can_always_move_back_inside():
    node = Harness()
    node.fence = Box((-1.0, -1.0, 0.0), (1.0, 1.0, 5.0))
    node._send(Setpoint(-0.5, 0.0, 0.0), Pose(0.9, 0.0, 1.0, 0.0), 0.05)
    assert node.backend.sent[-1][0] < 0.0


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_setpoint_never_reaches_the_fc(bad):
    node = Harness()
    node._last_setpoint = Setpoint(0.5, 0.0, 0.0)
    node._send(Setpoint(bad, 0.0, 0.0, bad), Pose(0.0, 0.0, 1.0, 0.0), 0.05)
    assert all(math.isfinite(v) for v in node.backend.sent[-1])
    assert all(math.isfinite(v) for v in vars(node._last_setpoint).values())


@pytest.mark.parametrize(
    "goal",
    [
        goto_goal(x=math.nan),
        goto_goal(z=math.inf),
        goto_goal(max_speed_mps=math.nan),
        goto_goal(max_speed_mps=-1.0),  # would disable the speed limit in goto_setpoint
        goto_goal(position_tolerance_m=math.inf),
        goto_goal(yaw_tolerance_rad=-0.1),
    ],
)
def test_goto_rejects_invalid_goals(goal):
    assert Harness()._goal_goto(goal) == GoalResponse.REJECT


def test_goto_rejects_bad_orientation():
    goal = goto_goal()
    goal.target.pose.orientation.w = 0.0
    assert Harness()._goal_goto(goal) == GoalResponse.REJECT
    goal.target.pose.orientation.w = math.nan
    assert Harness()._goal_goto(goal) == GoalResponse.REJECT


def test_goto_accepts_a_valid_goal():
    assert Harness()._goal_goto(goto_goal(max_speed_mps=0.5)) == GoalResponse.ACCEPT


@pytest.mark.parametrize("duration", [math.nan, math.inf, -1.0])
def test_hover_rejects_invalid_durations(duration):
    assert Harness()._goal_hover(Hover.Goal(duration_s=duration)) == GoalResponse.REJECT


def test_cmd_vel_drops_nonfinite_and_unknown_frames():
    node = Harness()
    msg = TwistStamped()
    msg.twist.linear.x = math.nan
    node._on_cmd_vel(msg)
    assert node.active is None
    msg = TwistStamped()
    msg.header.frame_id = "camera_color"
    node._on_cmd_vel(msg)
    assert node.active is None
    msg.header.frame_id = "base_link"
    node._on_cmd_vel(msg)
    assert node.active == "velocity"
