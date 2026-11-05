# Is the corridor ahead of the drone (the next route segment) free? Published on /mapping/blocked.
# Phase 1 only uses this as a "stop and hold" signal; there is no replanning.
std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_CLEAR=1
uint8 STATE_BLOCKED=2

# Mapping backend name: none | nvblox
string backend
uint8 state
# Convenience flag, equal to (state == STATE_BLOCKED).
bool blocked
# Smallest obstacle distance along the checked segment, metres. NaN when unknown.
float32 clearance_m
string message
