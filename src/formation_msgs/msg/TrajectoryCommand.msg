# =============================================================================
# COMMUNICATION TYPE: EXTERNAL (Can be serialized for inter-drone communication)
# =============================================================================
# Purpose: formation_manager sends individual trajectory commands to each drone
# Flow: formation_manager → /V{N}/trajectory_command → trajectory_planner
# Note: Each drone receives its own personal target and waypoints
# =============================================================================

std_msgs/Header header

# Target drone identification
int32 drone_id

# Mission sequencing
int32 sequence
string mission_id

# Individual target for this specific drone
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity

# Individual waypoints for this specific drone
geometry_msgs/Point[] waypoints

# Formation metadata (for optimizer)
string formation_type
float64 formation_scale
geometry_msgs/Vector3 formation_offset
