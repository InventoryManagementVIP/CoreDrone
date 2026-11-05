# Aggregated health from inv_safety. Published on /safety/status.
std_msgs/Header header

uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ABORT=2

uint8 level
# Human-readable reasons for WARN/ABORT, for example "battery 18% < 25%".
string[] reasons
# True once an abort (hold, then land) has been requested from the flight interface.
bool abort_requested
