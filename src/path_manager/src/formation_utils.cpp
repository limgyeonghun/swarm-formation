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

    if (formation_type == "square") {
        // Square/rectangular grid formation
        // Try to make it as square-like as possible
        int cols = static_cast<int>(std::ceil(std::sqrt(num_drones)));
        int rows = static_cast<int>(std::ceil(static_cast<double>(num_drones) / cols));

        double x_spacing = scale / (cols - 1);
        double y_spacing = scale / (rows - 1);

        int drone_idx = 0;
        for (int row = 0; row < rows && drone_idx < num_drones; ++row) {
            for (int col = 0; col < cols && drone_idx < num_drones; ++col) {
                double x = -scale/2 + col * x_spacing;
                double y = -scale/2 + row * y_spacing;
                pattern.push_back(Eigen::Vector3d(x, y, 0.0));
                drone_idx++;
            }
        }
    }
    else if (formation_type == "square_wide") {
        // Tall rectangular grid formation (y-axis is longer than x-axis)
        // Ordered to match circle formation: top-right, bottom-right, bottom-left, top-left
        int cols = static_cast<int>(std::ceil(std::sqrt(num_drones)));
        int rows = static_cast<int>(std::ceil(static_cast<double>(num_drones) / cols));

        double x_scale = scale;
        double y_scale = scale * 1.8;  // Make y-axis 1.8x longer

        double x_spacing = (cols > 1) ? x_scale / (cols - 1) : 0.0;
        double y_spacing = (rows > 1) ? y_scale / (rows - 1) : 0.0;

        // Create grid positions matching circle order
        // For 4 drones (2x2): top-right, bottom-right, bottom-left, top-left
        std::vector<std::pair<int, int>> grid_order;
        if (num_drones == 4) {
            // Match circle: [45°, -45°, -135°, -225°] = [top-right, bottom-right, bottom-left, top-left]
            grid_order = {{1, 1}, {0, 1}, {0, 0}, {1, 0}};  // {row, col}
        } else {
            // Default grid order for other drone counts
            for (int row = 0; row < rows; ++row) {
                for (int col = 0; col < cols; ++col) {
                    grid_order.push_back({row, col});
                    if (grid_order.size() >= num_drones) break;
                }
                if (grid_order.size() >= num_drones) break;
            }
        }

        for (const auto& pos : grid_order) {
            int row = pos.first;
            int col = pos.second;
            double x = -x_scale/2 + col * x_spacing;
            double y = -y_scale/2 + row * y_spacing;
            pattern.push_back(Eigen::Vector3d(x, y, 0.0));
        }
    }
    else if (formation_type == "triangle" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Triangle vertices: 3 corners (always present)
        // Apex pointing in travel direction (-y)
        pattern.push_back(Eigen::Vector3d(0.0, -2.0*h/3.0, 0.0));         // Bottom (apex, pointing forward)
        pattern.push_back(Eigen::Vector3d(-scale/2.0, h/3.0, 0.0));       // Top left
        pattern.push_back(Eigen::Vector3d(+scale/2.0, h/3.0, 0.0));       // Top right

        // Distribute remaining drones based on total count
        if (num_drones == 4) {
            // 4 drones: 3 vertices + 1 center
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        else if (num_drones == 6) {
            // 6 drones: 3 vertices + 3 edge midpoints (symmetric)
            pattern.push_back(Eigen::Vector3d(0.0, -h/3.0, 0.0));          // Bottom edge midpoint (apex side)
            pattern.push_back(Eigen::Vector3d(-scale/4.0, h/3.0, 0.0));    // Top edge left-mid
            pattern.push_back(Eigen::Vector3d(+scale/4.0, h/3.0, 0.0));    // Top edge right-mid
        }
        else if (num_drones == 8) {
            // 8 drones: 3 vertices + 1 center + 4 on edges (2 on top, 1 on each side)
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));             // Center
            pattern.push_back(Eigen::Vector3d(-scale/3.0, h/3.0, 0.0));    // Top edge left
            pattern.push_back(Eigen::Vector3d(+scale/3.0, h/3.0, 0.0));    // Top edge right
            pattern.push_back(Eigen::Vector3d(-scale/4.0, -h/6.0, 0.0));   // Left edge mid
            pattern.push_back(Eigen::Vector3d(+scale/4.0, -h/6.0, 0.0));   // Right edge mid
        }
        else if (num_drones == 10) {
            // 10 drones: 3 vertices + 1 center + 6 on edges (2 per edge, symmetric)
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));             // Center
            // Top edge: 2 drones
            pattern.push_back(Eigen::Vector3d(-scale/3.0, h/3.0, 0.0));
            pattern.push_back(Eigen::Vector3d(+scale/3.0, h/3.0, 0.0));
            // Left edge: 2 drones
            pattern.push_back(Eigen::Vector3d(-scale/4.0, -h/6.0, 0.0));
            pattern.push_back(Eigen::Vector3d(-scale/6.0, -h/3.0, 0.0));
            // Right edge: 2 drones
            pattern.push_back(Eigen::Vector3d(+scale/4.0, -h/6.0, 0.0));
            pattern.push_back(Eigen::Vector3d(+scale/6.0, -h/3.0, 0.0));
        }
        else if (num_drones > 10) {
            // Fallback: center + distribute remaining along edges
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            int remaining = num_drones - 4;
            int per_edge = remaining / 3;

            for (int i = 0; i < per_edge; ++i) {
                double t = (i + 1.0) / (per_edge + 1.0);
                // Top edge (now base)
                pattern.push_back(Eigen::Vector3d(-scale/2.0 + t * scale, h/3.0, 0.0));
            }
            for (int i = 0; i < per_edge; ++i) {
                double t = (i + 1.0) / (per_edge + 1.0);
                // Left edge
                pattern.push_back(Eigen::Vector3d(
                    -scale/2.0 * (1.0 - t),
                    h/3.0 - t * h,
                    0.0
                ));
            }
            for (int i = 0; i < remaining - 2 * per_edge; ++i) {
                double t = (i + 1.0) / (remaining - 2 * per_edge + 1.0);
                // Right edge
                pattern.push_back(Eigen::Vector3d(
                    scale/2.0 * (1.0 - t),
                    h/3.0 - t * h,
                    0.0
                ));
            }
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
    else if (formation_type == "wedge" || formation_type == "v_formation") {
        // V-shaped wedge formation pointing in travel direction (-y)
        // Scale represents the width at the back
        // Formation shape: > (pointing forward in -y direction)

        if (num_drones == 1) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        } else {
            // Calculate positions along two diagonal lines forming a V
            bool is_odd = (num_drones % 2 == 1);
            int side_count = num_drones / 2;  // Number per side (excluding center for odd)

            double angle = 60.0 * M_PI / 180.0;  // 60 degree angle from center (wider V)
            double spacing = scale / side_count;  // spacing along each arm

            if (is_odd) {
                // Odd number: center point at origin
                pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            }

            // Left arm (negative x, positive y - backward)
            for (int i = 1; i <= side_count; ++i) {
                double dist = i * spacing;
                pattern.push_back(Eigen::Vector3d(
                    -dist * sin(angle),  // negative x
                    dist * cos(angle),   // positive y (backward, wide formation)
                    0.0
                ));
            }

            // Right arm (positive x, positive y - backward)
            for (int i = 1; i <= side_count; ++i) {
                double dist = i * spacing;
                pattern.push_back(Eigen::Vector3d(
                    dist * sin(angle),   // positive x
                    dist * cos(angle),   // positive y (backward, wide formation)
                    0.0
                ));
            }
        }
    }
    else if (formation_type == "square_perimeter") {
        // Special case for 6 drones: symmetric right 3, left 3
        if (num_drones == 6) {
            pattern.push_back(Eigen::Vector3d(scale/2, scale/2, 0.0));    // Right-top corner
            pattern.push_back(Eigen::Vector3d(scale/2, 0.0, 0.0));        // Right middle
            pattern.push_back(Eigen::Vector3d(scale/2, -scale/2, 0.0));   // Right-bottom corner
            pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));  // Left-bottom corner
            pattern.push_back(Eigen::Vector3d(-scale/2, 0.0, 0.0));       // Left middle
            pattern.push_back(Eigen::Vector3d(-scale/2, scale/2, 0.0));   // Left-top corner
        }
        // Special case for 10 drones: asymmetric to prevent Hungarian flip (right 3, bottom 2, left 3, top 2)
        else if (num_drones == 10) {
            // Right edge: 3 drones (top to bottom)
            pattern.push_back(Eigen::Vector3d(scale/2, scale/2, 0.0));    // 0: Right-top corner
            pattern.push_back(Eigen::Vector3d(scale/2, 0.0, 0.0));        // 1: Right center
            pattern.push_back(Eigen::Vector3d(scale/2, -scale/2, 0.0));   // 2: Right-bottom corner

            // Bottom edge: 2 drones (not including corners)
            pattern.push_back(Eigen::Vector3d(scale/4, -scale/2, 0.0));   // 3: Bottom right-mid
            pattern.push_back(Eigen::Vector3d(-scale/4, -scale/2, 0.0));  // 4: Bottom left-mid

            // Left edge: 3 drones (bottom to top)
            pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));  // 5: Left-bottom corner
            pattern.push_back(Eigen::Vector3d(-scale/2, 0.0, 0.0));       // 6: Left center
            pattern.push_back(Eigen::Vector3d(-scale/2, scale/2, 0.0));   // 7: Left-top corner

            // Top edge: 2 drones (not including corners)
            pattern.push_back(Eigen::Vector3d(-scale/4, scale/2, 0.0));   // 8: Top left-mid
            pattern.push_back(Eigen::Vector3d(scale/4, scale/2, 0.0));    // 9: Top right-mid
        }
        else {
            // Square perimeter with GUARANTEED corners
            // Distribute drones per edge to ensure corners are occupied.
            // N=10 -> Sides get [3, 3, 2, 2] drones respectively.

            int drones_per_side = num_drones / 4;
            int remainder = num_drones % 4;

            int current_drone = 0;

            // Define corners: TR, BR, BL, TL
            double corners_x[5] = {scale/2,  scale/2, -scale/2, -scale/2, scale/2};
            double corners_y[5] = {scale/2, -scale/2, -scale/2,  scale/2, scale/2};

            for (int side = 0; side < 4; ++side) {
                // Calculate how many drones on this specific side
                int count = drones_per_side + (side < remainder ? 1 : 0);

                // Start and End points for this side
                double start_x = corners_x[side];
                double start_y = corners_y[side];
                double end_x = corners_x[side + 1];
                double end_y = corners_y[side + 1];

                for (int i = 0; i < count; ++i) {
                    // Linear interpolation: P = Start + t * (End - Start)
                    // t goes from 0 to (count-1)/count.
                    // We assume the start of the NEXT side covers the full '1.0' case (the next corner)
                    double t = (double)i / count;

                    double x = start_x + t * (end_x - start_x);
                    double y = start_y + t * (end_y - start_y);

                    pattern.push_back(Eigen::Vector3d(x, y, 0.0));
                    current_drone++;
                }
            }
        }
    }
    else if (formation_type == "circle") {
        double angle_step = 2.0 * M_PI / num_drones;

        double offset = angle_step / 2.0;

        for (int i = 0; i < num_drones; ++i) {
            double angle = -i * angle_step + offset;

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
