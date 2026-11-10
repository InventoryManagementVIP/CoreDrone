"""SafetyMonitor's input handling, driven without a ROS graph: silent publishers and lost aborts."""

from types import SimpleNamespace as NS

from builtin_interfaces.msg import Time

from inv_interfaces.msg import FlightState, LocalizationStatus
from inv_safety.monitor import SafetyMonitor
from inv_safety.policy import ABORT, OK, SafetyConfig


class FakeFuture:
    def __init__(self):
        self.callbacks = []
        self.finished = False

    def done(self):
        return self.finished

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class FakeClient:
    def __init__(self):
        self.ready = True
        self.requests = []
        self.removed = []
        self.future = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.future = FakeFuture()
        return self.future

    def remove_pending_request(self, future):
        self.removed.append(future)


class Harness(SafetyMonitor):
    def __init__(self):  # no super().__init__(): nothing here touches the ROS graph
        self.cfg = SafetyConfig()
        self.t = 100.0
        self._flight = self._flight_s = self._loc = self._loc_s = None
        self._loc_lost_since = self._link_lost_since = self._corridor = None
        self._abort_requested = False
        self._abort_reason = ""
        self._abort_future = None
        self._last_abort_attempt_s = float("-inf")
        self._last_level = 0
        self.abort_client = FakeClient()
        self.statuses = []
        self.status_pub = NS(publish=self.statuses.append)

    def get_logger(self):
        ignore = lambda *_a, **_k: None  # noqa: E731
        return NS(info=ignore, warning=ignore, error=ignore)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=Time))

    def _now(self):
        return self.t

    def feed_flight(self, **fields):
        base = {"connected": True, "armed": True, "flying": True, "battery_percentage": 0.8}
        self._on_flight(FlightState(**{**base, **fields}))

    def feed_loc(self, state=LocalizationStatus.STATE_TRACKING):
        self._on_loc(LocalizationStatus(state=state))

    def advance(self, seconds, flight=None, loc=True, dt=0.2):
        for _ in range(round(seconds / dt)):
            self.t += dt
            self.feed_flight(**(flight or {}))
            if loc:
                self.feed_loc()
            self._tick()


def test_all_healthy_is_ok():
    node = Harness()
    node.advance(2.0)
    assert node.statuses[-1].level == OK and not node.statuses[-1].abort_requested


def test_silent_localization_publisher_aborts():
    node = Harness()
    node.advance(1.0)
    node.advance(1.9, loc=False)  # within localization_stale_s + localization_lost_grace_s
    assert node.statuses[-1].level == OK
    node.advance(0.5, loc=False)
    assert node.statuses[-1].level == ABORT and node.statuses[-1].abort_requested


def test_localization_status_returning_clears_the_loss():
    node = Harness()
    node.advance(1.0)
    node.advance(1.2, loc=False)
    node.advance(1.0)
    assert node.statuses[-1].level == OK and not node.abort_client.requests


def _trigger_abort(node):
    node.advance(0.4, flight={"battery_percentage": 0.1})
    assert node.statuses[-1].abort_requested
    assert len(node.abort_client.requests) == 1


def test_lost_abort_is_resent_and_replaces_the_pending_request():
    node = Harness()
    _trigger_abort(node)
    first = node.abort_client.future
    node.advance(1.2, flight={"battery_percentage": 0.1})
    assert len(node.abort_client.requests) >= 2
    assert first in node.abort_client.removed


def test_rejected_abort_is_resent():
    node = Harness()
    _trigger_abort(node)
    node.abort_client.future.finished = True  # answered, but the vehicle is not landing
    node.advance(1.2, flight={"battery_percentage": 0.1})
    assert len(node.abort_client.requests) >= 2


def test_no_resend_once_the_flight_interface_is_landing():
    node = Harness()
    _trigger_abort(node)
    node.advance(3.0, flight={"battery_percentage": 0.1, "active_command": "land"})
    assert len(node.abort_client.requests) == 1


def test_abort_stays_latched_after_the_condition_clears():
    node = Harness()
    _trigger_abort(node)
    node.advance(1.2)  # battery reading recovers; the vehicle is still not landing
    assert node.statuses[-1].abort_requested
    assert len(node.abort_client.requests) >= 2


def test_pilot_override_stops_the_resends():
    node = Harness()
    _trigger_abort(node)
    node.advance(3.0, flight={"battery_percentage": 0.1, "pilot_override": True})
    assert len(node.abort_client.requests) == 1


def test_disarm_clears_the_latch():
    node = Harness()
    _trigger_abort(node)
    node.advance(0.4, flight={"armed": False, "flying": False})
    assert not node.statuses[-1].abort_requested


def test_abort_is_wanted_even_when_the_service_is_gone():
    node = Harness()
    node.abort_client.ready = False
    node.advance(0.4, flight={"battery_percentage": 0.1})
    assert node.statuses[-1].abort_requested  # the mission must still stop
    assert not node.abort_client.requests
    node.abort_client.ready = True
    node.advance(1.2, flight={"battery_percentage": 0.1})
    assert node.abort_client.requests
