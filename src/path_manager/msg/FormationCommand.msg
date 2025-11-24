# =============================================================================
# COMMUNICATION TYPE: EXTERNAL (Serial - to be implemented)
# =============================================================================
# Purpose: Commander sends formation change commands to all drones
# Flow: FormationCommander → formation_command → all FSMs
# Size: ~172 bytes (waypoints=3-5, with optimization)
# Note: Will be transmitted via serial in real deployment
# =============================================================================

std_msgs/Header header
string formation_type
float64 formation_scale
geometry_msgs/Point[] waypoints

# Centralized Hungarian algorithm results
# target_assignments[i] = target index for drone i
# Example: [0, 3, 1, 2] means drone0→target0, drone1→target3, drone2→target1, drone3→target2
int32[] target_assignments
