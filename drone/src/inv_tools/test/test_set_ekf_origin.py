import time
from dataclasses import dataclass

from pymavlink import mavutil

from inv_tools.set_ekf_origin import wait_autopilot_heartbeat

QUAD, GCS = mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_TYPE_GCS
ARDUPILOT, NO_AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavutil.mavlink.MAV_AUTOPILOT_INVALID


@dataclass
class Heartbeat:
    sysid: int
    compid: int
    type: int
    autopilot: int

    # pymavlink's message API
    def get_srcSystem(self):
        return self.sysid

    def get_srcComponent(self):
        return self.compid


class FakeMaster:
    def __init__(self, heartbeats):
        self.heartbeats = list(heartbeats)
        self.target_system = self.target_component = 0

    def recv_match(self, type, blocking, timeout):
        if self.heartbeats:
            return self.heartbeats.pop(0)
        time.sleep(timeout)  # like pymavlink: block until the timeout
        return None


def test_skips_forwarded_gcs_and_unconfigured_heartbeats():
    master = FakeMaster(
        [
            Heartbeat(255, 230, GCS, NO_AUTOPILOT),  # MAVProxy's heartbeat, forwarded by the FC
            Heartbeat(0, 1, QUAD, ARDUPILOT),
            Heartbeat(1, 1, QUAD, ARDUPILOT),
        ]
    )
    msg = wait_autopilot_heartbeat(master, timeout_s=5.0)
    assert (msg.sysid, msg.compid) == (1, 1)
    assert (master.target_system, master.target_component) == (1, 1)


def test_returns_none_without_an_autopilot():
    master = FakeMaster([Heartbeat(255, 230, GCS, NO_AUTOPILOT)])
    assert wait_autopilot_heartbeat(master, timeout_s=0.5) is None
