# Health of the active localization backend. Published on /localization/status.
std_msgs/Header header

uint8 STATE_UNKNOWN=0
uint8 STATE_TRACKING=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3

# Backend name: groundtruth | isaac | rtabmap
string backend
uint8 state
# Rate of the backend's odometry over the last window, Hz.
float32 rate_hz
# Age of the newest odometry sample, seconds.
float32 age_s
# True while the adapter publishes odom->base_link on /localization/external_nav
# (inv_flight_interface forwards it to the flight controller).
bool publishing_external_nav
string message
