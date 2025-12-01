# =============================================================================
# COMMUNICATION TYPE: EXTERNAL (Serial - to be implemented)
# =============================================================================
# Purpose: Commander sends formation change commands to all drones
# Flow: FormationCommander → formation_command → all FSMs
# Size: ~172 bytes (waypoints=3-5, with optimization)
# Note: Will be transmitted via serial in real deployment
# =============================================================================

std_msgs/Header header

# Mission sequencing for robustness
int32 sequence                # Incrementing sequence number (0, 1, 2, ...) for duplicate detection
string current_mission_id     # Current mission identifier (e.g., "waypoint_1", "formation_A")
string next_mission_id        # Next mission identifier (e.g., "waypoint_2", "MISSION_END")
bool is_final                 # True if this is the last mission

# Formation specification
string formation_type
float64 formation_scale
geometry_msgs/Point[] waypoints

# Centralized Hungarian algorithm results
# target_assignments[i] = target index for drone i
# Example: [0, 3, 1, 2] means drone0→target0, drone1→target3, drone2→target1, drone3→target2
int32[] target_assignments
