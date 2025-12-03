# =============================================================================
# COMMUNICATION TYPE: EXTERNAL (Serial via j_fi)
# =============================================================================
# Purpose: Broadcast drone trajectory to other drones for collision avoidance
# Flow: FSM → broadcast_traj_send → j_fi (serial) → other drones
# Size: ~243 bytes (piece_num=3)
# Critical: Used for swarm collision avoidance
# =============================================================================

int16 drone_id
int32 traj_id
builtin_interfaces/Time start_time

uint8 order
float32[] coef_x
float32[] coef_y
float32[] coef_z
float32[] duration

# Flag to indicate if this is the final mission (for disarm decision)
bool is_final_mission
