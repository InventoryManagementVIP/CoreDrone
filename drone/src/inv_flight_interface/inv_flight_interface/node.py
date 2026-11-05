"""inv_flight_interface: the one gateway between the autonomy stack and the flight controller.

API (docs/interfaces.md):
  actions   /flight/takeoff  /flight/goto  /flight/hover  /flight/land   (inv_interfaces/action/*)
  service   /flight/abort                                                (inv_interfaces/srv/Abort)
  topics    /flight/state (FlightState, out)
            /flight/cmd_vel (TwistStamped in map or base_link, in; watchdog-protected passthrough)
            /localization/external_nav (Odometry odom->base_link, in; forwarded to the FC)
            /mission/status (MissionStatus, in; used as the mission heartbeat)

Design:
  * One 20 Hz control loop owns all FC commands. Action callbacks only register a Command and wait.
  * GoTo/Hover are closed-loop on our own localization (TF map->base_link). The FC receives
    velocity setpoints in GUIDED mode; it holds position by itself if they stop (GUID_TIMEOUT).
  * After any goal the vehicle keeps holding its last target until the next command.
  * If someone else changes the mode (RC safety pilot, GCS, FC failsafe), the stack stops
    commanding immediately and reports pilot_override until the vehicle disarms.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from inv_flight_interface.ap_dds_backend import ApDdsBackend
from inv_flight_interface.backend import FcState, FlightBackend, Request
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
from inv_flight_interface.copter_modes import mode_id, mode_name
from inv_interfaces.action import GoTo, Hover, Land, Takeoff
from inv_interfaces.msg import FlightState, MissionStatus
from inv_interfaces.srv import Abort

GUIDED = mode_id("GUIDED")
LAND = mode_id("LAND")

# Parameter defaults. config/flight_interface.yaml must match (test/test_config.py checks it).
DEFAULTS: dict[str, object] = {
    "backend": "ap_dds",
    "fc_namespace": "/ap",
    "control_rate_hz": 20.0,
    "map_frame": "map",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "link_timeout_s": 1.0,
    "max_horizontal_speed_mps": 1.0,
    "max_vertical_speed_mps": 0.5,
    "max_yaw_rate_rad_s": 0.8,
    "max_accel_mps2": 1.5,
    "gain_xy": 0.8,
    "gain_z": 1.0,
    "gain_yaw": 1.2,
    "goto_position_tolerance_m": 0.15,
    "goto_yaw_tolerance_rad": 0.1,
    "goto_settle_s": 0.5,
    "goto_timeout_s": 90.0,
    "takeoff_altitude_tolerance_m": 0.15,
    "takeoff_timeout_s": 120.0,
    "request_retry_s": 2.0,
    "land_timeout_s": 90.0,
    "max_altitude_m": 4.0,
    "geofence_enabled": True,
    "geofence_min_xyz": [-50.0, -50.0, -1.0],
    "geofence_max_xyz": [50.0, 50.0, 5.0],
    "geofence_margin_m": 0.3,
    "cmd_vel_timeout_s": 0.5,
    "cmd_vel_hold_s": 2.0,
    "mission_heartbeat_timeout_s": 0.0,
    "mission_lost_land_s": 30.0,
    "abort_hold_s": 2.0,
    "external_nav_timeout_s": 0.5,
    "require_external_nav": True,
}


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass
class Command:
    kind: str  # takeoff | goto | hover | land | abort | velocity
    params: dict = field(default_factory=dict)
    done: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    canceled: bool = False
    message: str = ""
    phase: str = "start"
    started_s: float = 0.0
    phase_started_s: float = 0.0
    request: Request | None = None
    last_request_s: float = -1.0e9
    settle_since_s: float | None = None
    feedback: dict = field(default_factory=dict)


class FlightInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("flight_interface")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}

        self._lock = threading.RLock()
        self._cb_control = MutuallyExclusiveCallbackGroup()
        self._cb_actions = ReentrantCallbackGroup()
        self._cb_io = ReentrantCallbackGroup()

        if self.p["backend"] != "ap_dds":
            raise ValueError(f"unsupported backend {self.p['backend']!r} (only ap_dds; MAVROS is the planned fallback)")
        self.backend: FlightBackend = ApDdsBackend(self, self._cb_io, self.p["fc_namespace"])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.alignment = FrameAlignment()

        self.limits = Limits(
            max_horizontal_speed=self.p["max_horizontal_speed_mps"],
            max_vertical_speed=self.p["max_vertical_speed_mps"],
            max_yaw_rate=self.p["max_yaw_rate_rad_s"],
            max_accel=self.p["max_accel_mps2"],
        )
        self.gains = Gains(xy=self.p["gain_xy"], z=self.p["gain_z"], yaw=self.p["gain_yaw"])
        self.fence = Box(tuple(self.p["geofence_min_xyz"]), tuple(self.p["geofence_max_xyz"]))

        # Shared state, guarded by self._lock.
        self._cmd: Command | None = None
        self._hold: Pose | None = None
        self._takeoff_floor_z: float | None = None
        self._pilot_override = False
        self._in_control = False
        self._was_armed = False
        self._last_setpoint = Setpoint()
        self._last_tick_s: float | None = None
        self._last_external_nav_s: float | None = None
        self._last_odom_yaw: float | None = None
        self._last_mission_status_s: float | None = None
        self._mission_warned = False
        self._last_cmd_vel: TwistStamped | None = None
        self._last_cmd_vel_s: float | None = None

        self._state_pub = self.create_publisher(FlightState, "/flight/state", 10)
        self.create_subscription(
            Odometry,
            "/localization/external_nav",
            self._on_external_nav,
            qos_profile_sensor_data,
            callback_group=self._cb_io,
        )
        self.create_subscription(
            MissionStatus, "/mission/status", self._on_mission_status, 10, callback_group=self._cb_io
        )
        self.create_subscription(TwistStamped, "/flight/cmd_vel", self._on_cmd_vel, 10, callback_group=self._cb_io)

        def server(action_type, name, execute, goal_cb):
            return ActionServer(
                self,
                action_type,
                name,
                execute_callback=execute,
                goal_callback=goal_cb,
                cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                callback_group=self._cb_actions,
            )

        self._servers = [
            server(Takeoff, "/flight/takeoff", self._exec_takeoff, self._goal_takeoff),
            server(GoTo, "/flight/goto", self._exec_goto, self._goal_goto),
            server(Hover, "/flight/hover", self._exec_hover, self._goal_flying),
            server(Land, "/flight/land", self._exec_land, lambda _g: GoalResponse.ACCEPT),
        ]
        self.create_service(Abort, "/flight/abort", self._on_abort, callback_group=self._cb_actions)
        self.create_timer(1.0 / self.p["control_rate_hz"], self._tick, callback_group=self._cb_control)
        self.get_logger().info(f"flight interface up (backend={self.backend.name})")

    # ---- helpers -----------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None

    def _pose(self) -> Pose | None:
        t = self._lookup(self.p["map_frame"], self.p["base_frame"])
        if t is None:
            return None
        q, p = t.transform.rotation, t.transform.translation
        return Pose(p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _map_to_odom_yaw(self) -> float:
        t = self._lookup(self.p["map_frame"], self.p["odom_frame"])
        if t is None:
            return 0.0
        q = t.transform.rotation
        return yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _connected(self, st: FcState, now: float) -> bool:
        return st.last_message_s is not None and now - st.last_message_s < self.p["link_timeout_s"]

    def _external_nav_ok(self, now: float) -> bool:
        return (
            self._last_external_nav_s is not None and now - self._last_external_nav_s < self.p["external_nav_timeout_s"]
        )

    # ---- inputs -------------------------------------------------------------------------------
    def _on_external_nav(self, msg: Odometry) -> None:
        sent = self.backend.forward_external_nav(msg)
        q = msg.pose.pose.orientation
        with self._lock:
            self._last_odom_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if sent:
                self._last_external_nav_s = self._now()

    def _on_mission_status(self, _msg: MissionStatus) -> None:
        with self._lock:
            self._last_mission_status_s = self._now()
            self._mission_warned = False

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        with self._lock:
            st = self.backend.state()
            if self._pilot_override or not (st.armed and st.flying):
                return
            self._last_cmd_vel = msg
            self._last_cmd_vel_s = self._now()
            if self._cmd is None or self._cmd.kind != "velocity":
                if self._cmd is not None and self._cmd.kind in ("takeoff", "land", "abort"):
                    return  # never let streaming velocity preempt these
                self._start(Command("velocity"))

    # ---- command lifecycle ----------------------------------------------------------------------
    def _start(self, cmd: Command) -> None:
        """Make `cmd` the active command, preempting the current one. Caller holds the lock."""
        now = self._now()
        if self._cmd is not None and not self._cmd.done.is_set():
            self._finish(self._cmd, False, f"preempted by {cmd.kind}")
        cmd.started_s = cmd.phase_started_s = now
        self._cmd = cmd
        self._in_control = True

    def _finish(self, cmd: Command, success: bool, message: str) -> None:
        cmd.success, cmd.message = success, message
        cmd.done.set()
        if self._cmd is cmd:
            self._cmd = None
        # Two call sites: rclpy raises if one call site changes severity.
        text = f"{cmd.kind}: {'done' if success else 'failed'} ({message})"
        if success:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)

    def _phase(self, cmd: Command, phase: str) -> None:
        cmd.phase, cmd.phase_started_s = phase, self._now()
        cmd.request, cmd.last_request_s = None, -1.0e9

    def _retry_due(self, cmd: Command, now: float) -> bool:
        if cmd.request is not None and not cmd.request.done():
            return False
        return now - cmd.last_request_s >= self.p["request_retry_s"]

    # ---- actions --------------------------------------------------------------------------------
    def _goal_takeoff(self, goal) -> GoalResponse:
        st = self.backend.state()
        if self._pilot_override or (st.armed and st.flying):
            return GoalResponse.REJECT
        if not 0.3 <= goal.altitude_m <= self.p["max_altitude_m"]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _goal_flying(self, _goal) -> GoalResponse:
        st = self.backend.state()
        return GoalResponse.ACCEPT if st.armed and st.flying and not self._pilot_override else GoalResponse.REJECT

    def _goal_goto(self, goal) -> GoalResponse:
        if self._goal_flying(goal) != GoalResponse.ACCEPT:
            return GoalResponse.REJECT
        frame = goal.target.header.frame_id or self.p["map_frame"]
        if frame != self.p["map_frame"]:
            self.get_logger().warning(f"goto rejected: target frame {frame!r} is not {self.p['map_frame']!r}")
            return GoalResponse.REJECT
        p = goal.target.pose.position
        if self.p["geofence_enabled"] and not self.fence.contains((p.x, p.y, p.z), self.p["geofence_margin_m"]):
            self.get_logger().warning(f"goto rejected: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) is outside the geofence")
            return GoalResponse.REJECT
        if p.z > self.p["max_altitude_m"]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _run(self, goal_handle, cmd: Command, result_type, feedback_fn=None):
        with self._lock:
            self._start(cmd)
        while not cmd.done.wait(0.1):
            if not rclpy.ok():  # shutting down: the goal handle is no longer usable
                return result_type.Result()
            if goal_handle.is_cancel_requested:
                with self._lock:
                    if not cmd.done.is_set():
                        cmd.canceled = True
                        self._finish(cmd, False, "canceled")
                        pose = self._pose()
                        if pose is not None and cmd.kind in ("goto", "hover", "velocity"):
                            self._hold = pose
            elif feedback_fn is not None:
                fb = feedback_fn(cmd)
                if fb is not None:
                    try:
                        goal_handle.publish_feedback(fb)
                    except Exception:
                        if rclpy.ok():
                            raise
                        return result_type.Result()
        if cmd.canceled:
            goal_handle.canceled()
        elif cmd.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = result_type.Result()
        result.success, result.message = cmd.success, cmd.message
        if hasattr(result, "final_distance_m"):
            result.final_distance_m = float(cmd.feedback.get("distance_m", math.nan))
        return result

    def _exec_takeoff(self, goal_handle):
        def feedback(cmd):
            fb = Takeoff.Feedback()
            fb.altitude_m = float(cmd.feedback.get("altitude_m", 0.0))
            fb.phase = cmd.phase
            return fb

        cmd = Command("takeoff", {"altitude_m": float(goal_handle.request.altitude_m)})
        return self._run(goal_handle, cmd, Takeoff, feedback)

    def _exec_goto(self, goal_handle):
        req = goal_handle.request
        p, q = req.target.pose.position, req.target.pose.orientation
        params = {
            "target": (p.x, p.y, min(p.z, self.p["max_altitude_m"])),
            "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
            "max_speed": req.max_speed_mps or None,
            "tol": req.position_tolerance_m or self.p["goto_position_tolerance_m"],
            "yaw_tol": req.yaw_tolerance_rad or self.p["goto_yaw_tolerance_rad"],
        }

        def feedback(cmd):
            fb = GoTo.Feedback()
            fb.distance_m = float(cmd.feedback.get("distance_m", math.nan))
            fb.yaw_error_rad = float(cmd.feedback.get("yaw_error_rad", math.nan))
            return fb

        return self._run(goal_handle, Command("goto", params), GoTo, feedback)

    def _exec_hover(self, goal_handle):
        def feedback(cmd):
            fb = Hover.Feedback()
            fb.remaining_s = float(cmd.feedback.get("remaining_s", 0.0))
            return fb

        return self._run(
            goal_handle, Command("hover", {"duration_s": float(goal_handle.request.duration_s)}), Hover, feedback
        )

    def _exec_land(self, goal_handle):
        def feedback(cmd):
            fb = Land.Feedback()
            fb.altitude_m = float(cmd.feedback.get("altitude_m", math.nan))
            return fb

        return self._run(goal_handle, Command("land"), Land, feedback)

    def _on_abort(self, request: Abort.Request, response: Abort.Response) -> Abort.Response:
        with self._lock:
            st = self.backend.state()
            if not st.armed:
                response.accepted, response.message = False, "not armed"
                return response
            if self._pilot_override:
                response.accepted, response.message = False, "pilot has control"
                return response
            self.get_logger().warning(f"ABORT requested: {request.reason or 'no reason given'}")
            if self._cmd is not None and self._cmd.kind in ("land", "abort"):
                response.accepted, response.message = True, f"already {self._cmd.kind}ing"
                return response
            self._start(Command("abort", {"reason": request.reason}))
        response.accepted, response.message = True, "holding, then landing"
        return response

    # ---- control loop -----------------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        st = self.backend.state()
        connected = self._connected(st, now)
        pose = self._pose()
        with self._lock:
            dt = 0.0 if self._last_tick_s is None else max(0.0, now - self._last_tick_s)
            self._last_tick_s = now
            self._learn_alignment(st, now)
            self._check_pilot_override(st)
            self._watchdogs(st, pose, now)

            setpoint: Setpoint | None = None
            cmd = self._cmd
            if cmd is not None:
                setpoint = self._step(cmd, st, pose, connected, now)
            elif self._hold is not None and pose is not None and st.armed and st.flying:
                setpoint = self._hold_setpoint(pose, self._hold)
            if self._was_armed and not st.armed:
                # Flight over: forget targets; the next takeoff starts from a clean slate.
                self._hold = None
                self._in_control = self._cmd is not None
            self._was_armed = st.armed

            can_command = (
                setpoint is not None
                and connected
                and st.mode_id == GUIDED
                and not self._pilot_override
                and self.alignment.ready
                and pose is not None
            )
            if can_command:
                self._send(setpoint, pose, dt)
            else:
                self._last_setpoint = Setpoint()
            self._publish_state(st, connected, now)

    def _learn_alignment(self, st: FcState, now: float) -> None:
        if (
            st.fc_yaw_enu is not None
            and st.fc_yaw_stamp_s is not None
            and now - st.fc_yaw_stamp_s < 0.5
            and self._last_odom_yaw is not None
            and self._external_nav_ok(now)
            and st.ekf_origin_set
        ):
            was_ready = self.alignment.ready
            self.alignment.update(now, st.fc_yaw_enu, self._last_odom_yaw)
            if self.alignment.ready and not was_ready:
                self.get_logger().info(
                    f"FC map frame = odom rotated {math.degrees(self.alignment.rotation):+.0f} deg "
                    f"(residual {math.degrees(self.alignment.residual):+.1f} deg)"
                )

    def _check_pilot_override(self, st: FcState) -> None:
        if not st.armed:
            if self._pilot_override:
                self.get_logger().info("vehicle disarmed: pilot override cleared")
            self._pilot_override = False
            return
        expected = {GUIDED, LAND}
        if self._in_control and st.mode_id is not None and st.mode_id not in expected:
            if self._cmd is not None and self._cmd.kind == "takeoff" and self._cmd.phase in ("start", "mode"):
                return  # we have not switched to GUIDED yet
            if not self._pilot_override:
                self.get_logger().error(
                    f"mode changed to {mode_name(st.mode_id)} outside the stack: pilot override, "
                    "stopping all commands until disarmed"
                )
            self._pilot_override = True
            if self._cmd is not None:
                self._finish(self._cmd, False, f"pilot override ({mode_name(st.mode_id)})")
            self._hold = None

    def _watchdogs(self, st: FcState, pose: Pose | None, now: float) -> None:
        if not (st.armed and st.flying) or self._pilot_override:
            return
        # Mission heartbeat: hold, then land if the mission node disappears mid-flight.
        timeout = self.p["mission_heartbeat_timeout_s"]
        if timeout > 0.0 and self._last_mission_status_s is not None:
            age = now - self._last_mission_status_s
            landing = self._cmd is not None and self._cmd.kind in ("land", "abort")
            if age > self.p["mission_lost_land_s"] and not landing:
                self.get_logger().error(f"no mission heartbeat for {age:.0f} s: landing")
                self._start(Command("land"))
            elif age > timeout and not self._mission_warned:
                self._mission_warned = True
                self.get_logger().warning(f"no mission heartbeat for {age:.1f} s: holding position")
                if self._cmd is not None and self._cmd.kind in ("goto", "velocity"):
                    self._finish(self._cmd, False, "mission heartbeat lost")
                    if pose is not None:
                        self._hold = pose

    # ---- per-command steps ------------------------------------------------------------------
    def _step(self, cmd: Command, st: FcState, pose: Pose | None, connected: bool, now: float) -> Setpoint | None:
        return getattr(self, f"_step_{cmd.kind}")(cmd, st, pose, connected, now)

    def _hold_setpoint(self, pose: Pose, target: Pose) -> Setpoint:
        return goto_setpoint(pose.xyz, pose.yaw, target.xyz, target.yaw, self.gains, self.limits)

    def _step_takeoff(self, cmd, st, pose, connected, now):
        alt = cmd.params["altitude_m"]
        if now - cmd.started_s > self.p["takeoff_timeout_s"]:
            self._finish(cmd, False, f"timed out in phase {cmd.phase}")
            if st.armed:
                self._start(Command("land"))
            return None
        if cmd.phase == "start":
            reasons = []
            if not connected:
                reasons.append("no flight controller link")
            if not self.backend.services_ready():
                reasons.append("FC services not available")
            if pose is None:
                reasons.append("no localization (TF map->base_link)")
            if not st.ekf_origin_set:
                reasons.append("EKF origin not set")
            if self.p["require_external_nav"] and not self._external_nav_ok(now):
                reasons.append("external navigation not reaching the FC")
            if not self.alignment.ready:
                reasons.append("FC frame alignment not learned yet")
            cmd.feedback["waiting_for"] = ", ".join(reasons)
            if reasons:
                self.get_logger().info(f"takeoff waiting: {cmd.feedback['waiting_for']}", throttle_duration_sec=5.0)
                return None
            self._takeoff_floor_z = pose.z
            self._phase(cmd, "mode")
        if cmd.phase == "mode":
            if st.mode_id == GUIDED:
                self._phase(cmd, "arm")
            elif self._retry_due(cmd, now):
                cmd.request, cmd.last_request_s = self.backend.request_mode(GUIDED), now
        if cmd.phase == "arm":
            if st.armed:
                self._phase(cmd, "takeoff")
            elif self._retry_due(cmd, now):
                self.get_logger().info("arming", throttle_duration_sec=5.0)
                cmd.request, cmd.last_request_s = self.backend.request_arm(True), now
        if cmd.phase == "takeoff":
            if cmd.request is not None and cmd.request.done() and cmd.request.succeeded():
                self._phase(cmd, "climb")
            elif self._retry_due(cmd, now):
                cmd.request, cmd.last_request_s = self.backend.request_takeoff(alt), now
        if cmd.phase == "climb" and pose is not None and self._takeoff_floor_z is not None:
            height = pose.z - self._takeoff_floor_z
            cmd.feedback["altitude_m"] = height
            if height >= alt - self.p["takeoff_altitude_tolerance_m"]:
                self._hold = Pose(pose.x, pose.y, self._takeoff_floor_z + alt, pose.yaw)
                self._finish(cmd, True, f"at {height:.2f} m")
        return None  # the FC flies the takeoff itself

    def _step_goto(self, cmd, st, pose, connected, now):
        if pose is None:
            return None
        target, yaw = cmd.params["target"], cmd.params["yaw"]
        dist = math.dist(pose.xyz, target)
        yaw_err = wrap_angle(yaw - pose.yaw)
        cmd.feedback.update(distance_m=dist, yaw_error_rad=yaw_err)
        if dist < cmd.params["tol"] and abs(yaw_err) < cmd.params["yaw_tol"]:
            if cmd.settle_since_s is None:
                cmd.settle_since_s = now
            elif now - cmd.settle_since_s >= self.p["goto_settle_s"]:
                self._hold = Pose(*target, yaw)
                self._finish(cmd, True, f"reached within {dist:.2f} m")
        else:
            cmd.settle_since_s = None
        if not cmd.done.is_set() and now - cmd.started_s > self.p["goto_timeout_s"]:
            self._hold = pose
            self._finish(cmd, False, f"timed out {dist:.2f} m from target")
        return goto_setpoint(pose.xyz, pose.yaw, target, yaw, self.gains, self.limits, cmd.params["max_speed"])

    def _step_hover(self, cmd, st, pose, connected, now):
        if pose is None:
            return None
        if "anchor" not in cmd.params:
            held = self._hold
            near = held is not None and math.dist(held.xyz, pose.xyz) < 0.5
            cmd.params["anchor"] = held if near else pose
        remaining = cmd.params["duration_s"] - (now - cmd.started_s)
        cmd.feedback["remaining_s"] = max(0.0, remaining)
        if remaining <= 0.0:
            self._hold = cmd.params["anchor"]
            self._finish(cmd, True, f"hovered {cmd.params['duration_s']:.1f} s")
        return self._hold_setpoint(pose, cmd.params["anchor"])

    def _step_land(self, cmd, st, pose, connected, now):
        if pose is not None and self._takeoff_floor_z is not None:
            cmd.feedback["altitude_m"] = pose.z - self._takeoff_floor_z
        if not st.armed:
            self._hold = None
            self._finish(cmd, True, "landed and disarmed")
            return None
        if now - cmd.started_s > self.p["land_timeout_s"]:
            self._finish(cmd, False, "still armed after land timeout")
            return None
        if st.mode_id != LAND and self._retry_due(cmd, now):
            cmd.request, cmd.last_request_s = self.backend.request_mode(LAND), now
        self._hold = None
        return None

    def _step_abort(self, cmd, st, pose, connected, now):
        if "anchor" not in cmd.params and pose is not None:
            cmd.params["anchor"] = pose
        if now - cmd.started_s >= self.p["abort_hold_s"]:
            self._finish(cmd, True, "hold complete, landing")
            self._start(Command("land"))
            return None
        anchor = cmd.params.get("anchor")
        return self._hold_setpoint(pose, anchor) if (pose is not None and anchor is not None) else Setpoint()

    def _step_velocity(self, cmd, st, pose, connected, now):
        age = math.inf if self._last_cmd_vel_s is None else now - self._last_cmd_vel_s
        if age > self.p["cmd_vel_hold_s"]:
            self._hold = pose
            self._finish(cmd, True, "velocity stream stopped: holding position")
            return Setpoint()
        if age > self.p["cmd_vel_timeout_s"] or self._last_cmd_vel is None:
            return Setpoint()  # watchdog: stop, keep altitude
        t = self._last_cmd_vel.twist
        vx, vy = t.linear.x, t.linear.y
        if self._last_cmd_vel.header.frame_id == self.p["base_frame"] and pose is not None:
            vx, vy = rotate_xy(vx, vy, pose.yaw)
        return Setpoint(vx, vy, t.linear.z, t.angular.z)

    # ---- output -----------------------------------------------------------------------------
    def _send(self, setpoint: Setpoint, pose: Pose, dt: float) -> None:
        # Limits: speeds, geofence, ceiling, then acceleration.
        sp = Setpoint(
            *self._limit_speeds(setpoint.vx, setpoint.vy, setpoint.vz),
            max(-self.limits.max_yaw_rate, min(self.limits.max_yaw_rate, setpoint.yaw_rate)),
        )
        if self.p["geofence_enabled"]:
            vx, vy, vz = self.fence.limit_velocity(pose.xyz, (sp.vx, sp.vy, sp.vz), self.p["geofence_margin_m"])
            sp = Setpoint(vx, vy, vz, sp.yaw_rate)
        if pose.z >= self.p["max_altitude_m"] and sp.vz > 0.0:
            sp = Setpoint(sp.vx, sp.vy, 0.0, sp.yaw_rate)
        sp = slew(self._last_setpoint, sp, self.limits.max_accel, dt)
        self._last_setpoint = sp
        # map -> odom -> FC map frame (see FrameAlignment).
        ox, oy = rotate_xy(sp.vx, sp.vy, -self._map_to_odom_yaw())
        fx, fy = rotate_xy(ox, oy, self.alignment.rotation)
        self.backend.send_velocity(fx, fy, sp.vz, sp.yaw_rate)

    def _limit_speeds(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        h = math.hypot(vx, vy)
        if h > self.limits.max_horizontal_speed > 0.0:
            vx, vy = vx * self.limits.max_horizontal_speed / h, vy * self.limits.max_horizontal_speed / h
        vz = max(-self.limits.max_vertical_speed, min(self.limits.max_vertical_speed, vz))
        return vx, vy, vz

    def _publish_state(self, st: FcState, connected: bool, now: float) -> None:
        msg = FlightState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p["map_frame"]
        msg.connected = connected
        msg.link_age_s = math.inf if st.last_message_s is None else float(now - st.last_message_s)
        msg.armed, msg.flying = st.armed, st.flying
        msg.mode_id = st.mode_id if st.mode_id is not None else 255
        msg.mode = mode_name(st.mode_id)
        msg.pilot_override = self._pilot_override
        msg.ekf_origin_set = st.ekf_origin_set
        msg.external_nav_ok = self._external_nav_ok(now)
        msg.battery_voltage = float(st.battery_voltage)
        msg.battery_percentage = float(st.battery_percentage)
        if self._cmd is not None:
            msg.active_command = self._cmd.kind
        elif self._hold is not None and st.armed:
            msg.active_command = "hold"
        msg.failsafes = st.failsafes
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightInterfaceNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
