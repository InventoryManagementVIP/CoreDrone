# Flight controller state as seen by inv_flight_interface. Published on /flight/state.
# This is the only view of the flight controller that other packages get.
std_msgs/Header header

# Flight controller link (DDS) is alive.
bool connected
# Seconds since the last message from the flight controller.
float32 link_age_s

bool armed
bool flying
# Flight mode name, for example "GUIDED", "LAND", "LOITER".
string mode
uint8 mode_id

# True after someone else (RC safety pilot or ground station) changed the mode while the
# stack was in control. The stack then stops commanding until the vehicle is disarmed.
bool pilot_override

# EKF origin has been set (required for non-GPS flight).
bool ekf_origin_set
# External navigation (odom->base_link) is being forwarded to the flight controller.
bool external_nav_ok

# NaN when unknown. Percentage is a fraction in [0, 1], as in sensor_msgs/BatteryState.
float32 battery_voltage
float32 battery_percentage

# Command being executed: "", "takeoff", "goto", "hover", "land", "abort", "velocity", "hold".
string active_command
# Active flight-controller failsafes, as text.
string[] failsafes
