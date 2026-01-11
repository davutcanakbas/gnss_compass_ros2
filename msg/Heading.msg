# ArduSimple Dual-Antenna Heading Message
# True heading derived from dual GNSS antenna configuration

std_msgs/Header header

# True heading in degrees (0-360, clockwise from true north)
float64 heading_deg

# Heading in radians (-PI to PI)
float64 heading_rad

# Heading accuracy estimate (degrees, 1-sigma)
float32 heading_accuracy_deg

# Pitch angle from dual antenna (degrees, optional)
# Positive = front antenna higher than rear
float64 pitch_deg

# Baseline length between antennas (meters)
float32 baseline_length

# Quality indicators
bool heading_valid
bool baseline_fixed  # true if RTK fixed between antennas

# Raw relative position components (NED frame, meters)
float64 rel_pos_n  # North
float64 rel_pos_e  # East  
float64 rel_pos_d  # Down

