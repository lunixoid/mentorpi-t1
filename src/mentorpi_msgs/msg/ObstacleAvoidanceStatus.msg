# SD036 I1: obstacle avoidance state of motion_control (F12).
uint8 INACTIVE=0   # gate closed: not AutoFollow or target unusable
uint8 FREE=1       # SD025 command passes the bumper
uint8 AVOIDING=2   # maneuver instead of the SD025 command
uint8 BLOCKED=3    # every maneuver hits the bumper, command is zero
uint8 NO_SCAN=4    # no fresh scan or no TF: SD025 command without guard
uint8 DISABLED=5   # enable_avoidance is false

uint8 MANEUVER_NONE=0
uint8 MANEUVER_ARC=1
uint8 MANEUVER_ROTATE=2
uint8 MANEUVER_REVERSE=3

std_msgs/Header header
uint8 state
uint8 maneuver
float32 min_clearance   # m, footprint to nearest obstacle point now; -1 without points
float32 bumper_margin   # m, margin for the emitted |linear_x|
uint16 obstacle_points  # after self filter and person exclusion
uint16 excluded_points  # dropped by person exclusion
