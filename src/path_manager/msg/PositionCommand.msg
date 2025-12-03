# =============================================================================
# COMMUNICATION TYPE: INTERNAL (ROS2 topic only)
# =============================================================================
# Purpose: TrajServer sends position commands to RoverControl
# Flow: FSM → TrajServer → /agentN/target_position → RoverControl
# Size: ~80 bytes
# Note: NOT transmitted via serial - internal message within same drone
# Note: TrajServer samples PolyTraj at 10ms intervals and publishes this
# =============================================================================

std_msgs/Header header
geometry_msgs/Point position          # Target position (for velocity calculation)
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Point lookahead_point   # Lookahead point (for direction calculation)
float64 yaw
float64 yaw_dot

uint32 trajectory_id

uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7

# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag

# Flag to indicate if this is the final mission (for disarm decision)
bool is_final_mission