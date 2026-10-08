# schema 1: all spatial quantities use ENU/map and SI units.
std_msgs/Header header
uint32 schema_version
string node_instance_id
uint64 tick_sequence
uint64 flight_id
uint64 plan_id
float64 steady_time
# Callback-entry interval. 0 for the first callback.
float64 tick_interval
# Callback entry -> control/status/mode dispatch, before diagnostic serialization.
float64 tick_cost
geometry_msgs/Point actual_position
geometry_msgs/Vector3 actual_velocity
float64 actual_yaw
bool inputs_fresh
bool odom_valid
bool localization_reset
bool tick_clock_reset
float64 odom_stamp
# -1 means no accepted message. Negative otherwise reveals clock inconsistency.
float64 odom_receipt_age
float64 state_receipt_age
string input_failure_reason
string odometry_rejection_reason
string current_state
string fcu_mode
bool armed
string last_trigger
string last_trigger_reason
uint64 recorded_points
bool planning_requested
bool planning_inflight
bool trajectory_valid
bool fallback_valid
bool projection_valid
float64 projected_s
float64 target_s
float64 cross_track_error
float64 reference_time
float64 clock_rate
uint64 fallback_index
uint64 fallback_count
# Last safety evaluation; validity flags gate interpretation of cached values.
float64 route_safety_evaluation_time
bool route_projection_valid
float64 route_distance
bool route_prediction_valid
float64 route_predicted_deviation
bool route_safety_hold
bool input_hold
bool yaw_alignment_timeout
bool auto_land_yield
bool rc_loss
bool low_battery
# Latest operation retained in snapshots. Sequence increments when the operation reason changes.
uint64 operation_sequence
float64 operation_steady_time
string operation
# This is the prepared/dispatched target, not proof of FCU receipt or execution.
bool should_publish_setpoint
bool setpoint_present
bool setpoint_acceleration_present
# Prepared coordinates are ENU/map; coordinate_frame is the MAVROS request selector.
# The selector FRAME_LOCAL_NED does not make these diagnostic vector fields NED.
geometry_msgs/Point setpoint_position
geometry_msgs/Vector3 setpoint_velocity
geometry_msgs/Vector3 setpoint_acceleration
float64 setpoint_yaw
float64 setpoint_yaw_rate
uint16 setpoint_type_mask
uint8 setpoint_coordinate_frame
bool mode_request_present
string mode_request
