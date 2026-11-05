# Mission progress from inv_mission. Published on /mission/status at a steady rate; it doubles
# as the mission heartbeat that inv_flight_interface watches.
std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_WAITING_FOR_READY=1
uint8 STATE_TAKING_OFF=2
uint8 STATE_RUNNING=3
uint8 STATE_LANDING=4
uint8 STATE_COMPLETED=5
uint8 STATE_ABORTED=6
uint8 STATE_FAILED=7

string mission_id
string warehouse_id
uint8 state
# Index of the hover point being flown to (0-based) and how many the route has.
uint32 waypoint_index
uint32 waypoint_count
# Short description of the current behaviour, for example "goto hp-007".
string current_task
string message
