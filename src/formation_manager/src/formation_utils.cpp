#include "formation_manager/formation_utils.h"
#include <cmath>

namespace formation_manager {

std::vector<Eigen::Vector3d> FormationUtils::generateFormationPattern(
    const std::string& formation_type,
    int num_drones,
    double scale,
    double z_spacing)
{
    std::vector<Eigen::Vector3d> pattern;
    pattern.reserve(num_drones);

    if (formation_type == "square" && num_drones == 4) {
        // 3D square pyramid - two at base, two elevated
        pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));
        pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));
        pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, z_spacing));
        pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, z_spacing));
    }
    else if (formation_type == "triangle" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // 3D triangle - apex elevated
        pattern.push_back(Eigen::Vector3d(0.0, 2.0*h/3.0, z_spacing));   // Top (apex) elevated
        pattern.push_back(Eigen::Vector3d(-scale/2.0, -h/3.0, 0.0));      // Bottom left
        pattern.push_back(Eigen::Vector3d(+scale/2.0, -h/3.0, 0.0));      // Bottom right

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, z_spacing/2));    // Center mid-level
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
            double line_position = scale/2 - i * spacing;  // Start from +scale/2 and go down
            pattern.push_back(Eigen::Vector3d(
                line_position * cos(line_angle),
                line_position * sin(line_angle),
                0.0
            ));
        }
    }
    else if (formation_type == "line_first_reverse") {
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
    else if (formation_type == "line_test") {
        // Horizontal line perpendicular to y-axis (travel direction)
        double spacing = (num_drones > 1) ? scale / (num_drones - 1) : 0.0;

        for (int i = 0; i < num_drones; ++i) {
            double x_position = -scale/2 + i * spacing;  // Left to right along x-axis
            pattern.push_back(Eigen::Vector3d(
                x_position,
                0.0,
                0.0
            ));
        }
    }
    else if (formation_type == "circle") {
        double angle_step = 2.0 * M_PI / num_drones;
        for (int i = 0; i < num_drones; ++i) {
            double angle = i * angle_step;
            double z = (i % 2) * z_spacing;  // Alternating heights
            pattern.push_back(Eigen::Vector3d(
                scale * cos(angle),
                scale * sin(angle),
                z
            ));
        }
    }
    else if (formation_type == "none" || formation_type == "NONE") {
        // No formation mode - all drones go to same target point
        // Zero offset for all drones (formation disabled)
        for (int i = 0; i < num_drones; ++i) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
    }
    else {
        // Unknown formation type - fallback to square/circle
        if (num_drones <= 4) {
            pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));
            if (num_drones > 1) pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));
            if (num_drones > 2) pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, z_spacing));
            if (num_drones > 3) pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, z_spacing));
        } else {
            double angle_step = 2.0 * M_PI / num_drones;
            for (int i = 0; i < num_drones; ++i) {
                double angle = i * angle_step;
                double z = (i % 2) * z_spacing;
                pattern.push_back(Eigen::Vector3d(
                    scale * cos(angle),
                    scale * sin(angle),
                    z
                ));
            }
        }
    }

    return pattern;
}

} // namespace formation_manager
