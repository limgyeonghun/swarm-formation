#include "path_manager/formation_utils.h"
#include <cmath>

namespace path_manager {

std::vector<Eigen::Vector3d> FormationUtils::generateFormationPattern(
    const std::string& formation_type,
    int num_drones,
    double scale)
{
    std::vector<Eigen::Vector3d> pattern;
    pattern.reserve(num_drones);

    if (formation_type == "square" && num_drones == 4) {
        // Standard square formation
        pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));  // Drone 0: 왼쪽 아래
        pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));  // Drone 1: 왼쪽 위
        pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, 0.0));  // Drone 2: 오른쪽 위
        pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));  // Drone 3: 오른쪽 아래
    }
    else if (formation_type == "triangle" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Triangle vertices: each drone at a corner
        pattern.push_back(Eigen::Vector3d(0.0, 2.0*h/3.0, 0.0));          // Top (apex)
        pattern.push_back(Eigen::Vector3d(-scale/2.0, -h/3.0, 0.0));      // Bottom left
        pattern.push_back(Eigen::Vector3d(+scale/2.0, -h/3.0, 0.0));      // Bottom right

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));            // Center
        }
    }
    else if (formation_type == "triangle_rotated" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Rotated 90 degrees clockwise: apex points left (direction of travel)
        pattern.push_back(Eigen::Vector3d(-2.0*h/3.0, 0.0, 0.0));         // Left center (apex, direction of travel)
        pattern.push_back(Eigen::Vector3d(+h/3.0, +scale/2.0, 0.0));      // Right top
        pattern.push_back(Eigen::Vector3d(+h/3.0, 0.0, 0.0));             // Right center

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(+h/3.0, -scale/2.0, 0.0));  // Right bottom
        }
    }
    else if (formation_type == "line_first") {
        double spacing = (num_drones > 1) ? scale / (num_drones - 1) : 0.0;
        double line_angle = 83.0 * M_PI / 180.0;

        for (int i = 0; i < num_drones; ++i) {
            double line_position = -scale/2 + i * spacing;  // Start from +scale/2 and go down
            pattern.push_back(Eigen::Vector3d(
                line_position * cos(line_angle),
                line_position * sin(line_angle),
                0.0
            ));
        }
    }
    else if (formation_type == "line_first_no_offset") {
        // No offset version - all drones target same waypoint, formation maintained by local optimizer
        for (int i = 0; i < num_drones; ++i) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));  // Zero offset for all
        }
    }
    else if (formation_type == "line_second") {
        double spacing = (num_drones > 1) ? scale / (num_drones - 1) : 0.0;
        double line_angle = -8.63 * M_PI / 180.0;

        for (int i = 0; i < num_drones; ++i) {
            double line_position = -scale/2 + i * spacing;
            pattern.push_back(Eigen::Vector3d(
                line_position * cos(line_angle),
                line_position * sin(line_angle),
                0.0
            ));
        }
    }
    else if (formation_type == "line_second_no_offset") {
        // No offset version - all drones target same waypoint, formation maintained by local optimizer
        for (int i = 0; i < num_drones; ++i) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));  // Zero offset for all
        }
    }
    else if (formation_type == "circle") {
        double angle_step = 2.0 * M_PI / num_drones;
        for (int i = 0; i < num_drones; ++i) {
            double angle = i * angle_step;
            pattern.push_back(Eigen::Vector3d(
                scale * cos(angle),
                scale * sin(angle),
                0.0
            ));
        }
    }
    else {
        // Unknown formation type - fallback to square/circle
        if (num_drones <= 4) {
            pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));
            if (num_drones > 1) pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));
            if (num_drones > 2) pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, 0.0));
            if (num_drones > 3) pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));
        } else {
            double angle_step = 2.0 * M_PI / num_drones;
            for (int i = 0; i < num_drones; ++i) {
                double angle = i * angle_step;
                pattern.push_back(Eigen::Vector3d(
                    scale * cos(angle),
                    scale * sin(angle),
                    0.0
                ));
            }
        }
    }

    return pattern;
}

} // namespace path_manager
