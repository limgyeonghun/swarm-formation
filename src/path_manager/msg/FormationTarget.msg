# =============================================================================
# COMMUNICATION TYPE: INTERNAL (ROS2 topic only)
# =============================================================================
# Purpose: FSM publishes formation target to itself (internal use)
# Flow: FSM → formation_targets → same FSM (filtered by drone_id)
# Size: ~210 bytes
# Note: NOT transmitted via serial - internal message only
# Note: Each FSM only processes messages matching its own drone_id
# =============================================================================

std_msgs/Header header
int32 drone_id
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity
string formation_type
float64 formation_scale
geometry_msgs/Point[] formation_positions
geometry_msgs/Vector3 formation_offset

