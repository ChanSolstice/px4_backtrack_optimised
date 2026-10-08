# Frozen planning input and outcome. Input is original recording order, ENU.
# Reversal/tangent-yaw and half-acceleration budget are applied inside TrajectorySession.
std_msgs/Header header
uint32 schema_version
string node_instance_id
uint64 flight_id
uint64 plan_id
float64 steady_time_s
string phase
string status
string message
uint32 warning_flags
bool success
bool fallback_ready
float64 generation_cost_ms
float64 duration_s
float64 path_length_m
float64 max_speed_m_s
float64 max_yaw_rate_rad_s
float64 motion_speed_scale
bool motion_limits_verified
bool corridor_verified
string report
# Full input is present in every phase so late subscribers/rotated bags retain it.
geometry_msgs/Point[] raw_position
float64[] raw_yaw_rad
float64[] raw_timestamp_s
string effective_config
float64 acceleration_budget_scale
