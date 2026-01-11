# ArduSimple GNSS Status Message
# Provides detailed status information about GNSS fix quality

std_msgs/Header header

# Fix type enumeration
uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING = 1
uint8 FIX_TYPE_2D_FIX = 2
uint8 FIX_TYPE_3D_FIX = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING = 4
uint8 FIX_TYPE_TIME_ONLY = 5

# RTK status enumeration  
uint8 RTK_NONE = 0
uint8 RTK_FLOAT = 1
uint8 RTK_FIXED = 2

# Current fix type
uint8 fix_type

# RTK correction status
uint8 rtk_status

# Number of satellites used in solution
uint8 satellites_used

# Number of satellites visible
uint8 satellites_visible

# Horizontal dilution of precision
float32 hdop

# Vertical dilution of precision
float32 vdop

# Position dilution of precision
float32 pdop

# Horizontal accuracy estimate (meters, 1-sigma)
float32 horizontal_accuracy

# Vertical accuracy estimate (meters, 1-sigma)
float32 vertical_accuracy

# Speed accuracy estimate (m/s, 1-sigma)
float32 speed_accuracy

# Heading accuracy estimate (degrees, 1-sigma)
float32 heading_accuracy

# Age of differential corrections (seconds)
float32 diff_age

# Dual antenna baseline valid
bool baseline_valid

# Baseline length between antennas (meters)
float32 baseline_length

# Flags
bool gnss_fix_valid
bool differential_applied
bool heading_valid
bool carrier_solution_valid

