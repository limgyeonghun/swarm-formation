#include "path_manager/hungarian_algorithm.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>

namespace path_manager {

const double HungarianAlgorithm::INF = std::numeric_limits<double>::max() / 2.0;

std::vector<std::vector<double>> HungarianAlgorithm::createCostMatrix(
    const std::vector<Eigen::Vector3d>& current_positions,
    const std::vector<Eigen::Vector3d>& target_positions) {

    int n = current_positions.size();
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            // Cost is the Euclidean distance between current position i and target position j
            cost_matrix[i][j] = (current_positions[i] - target_positions[j]).norm();
        }
    }

    return cost_matrix;
}

std::vector<Eigen::Vector3d> HungarianAlgorithm::applyAssignment(
    const std::vector<int>& assignment,
    const std::vector<Eigen::Vector3d>& target_positions) {

    int n = assignment.size();
    std::vector<Eigen::Vector3d> reordered_targets(n);

    for (int i = 0; i < n; ++i) {
        reordered_targets[i] = target_positions[assignment[i]];
    }

    return reordered_targets;
}

void HungarianAlgorithm::reduceRows(std::vector<std::vector<double>>& cost, int n) {
    for (int i = 0; i < n; ++i) {
        double min_val = cost[i][0];
        for (int j = 1; j < n; ++j) {
            if (cost[i][j] < min_val) {
                min_val = cost[i][j];
            }
        }
        for (int j = 0; j < n; ++j) {
            cost[i][j] -= min_val;
        }
    }
}

void HungarianAlgorithm::reduceCols(std::vector<std::vector<double>>& cost, int n) {
    for (int j = 0; j < n; ++j) {
        double min_val = cost[0][j];
        for (int i = 1; i < n; ++i) {
            if (cost[i][j] < min_val) {
                min_val = cost[i][j];
            }
        }
        for (int i = 0; i < n; ++i) {
            cost[i][j] -= min_val;
        }
    }
}

bool HungarianAlgorithm::findUncoveredZero(const std::vector<std::vector<double>>& cost, int n,
                                            const std::vector<bool>& row_covered,
                                            const std::vector<bool>& col_covered,
                                            int& row, int& col) {
    for (int i = 0; i < n; ++i) {
        if (row_covered[i]) continue;
        for (int j = 0; j < n; ++j) {
            if (col_covered[j]) continue;
            if (std::abs(cost[i][j]) < 1e-9) {  // Check if zero (with floating point tolerance)
                row = i;
                col = j;
                return true;
            }
        }
    }
    return false;
}

double HungarianAlgorithm::findMinUncovered(const std::vector<std::vector<double>>& cost, int n,
                                             const std::vector<bool>& row_covered,
                                             const std::vector<bool>& col_covered) {
    double min_val = INF;
    for (int i = 0; i < n; ++i) {
        if (row_covered[i]) continue;
        for (int j = 0; j < n; ++j) {
            if (col_covered[j]) continue;
            if (cost[i][j] < min_val) {
                min_val = cost[i][j];
            }
        }
    }
    return min_val;
}

std::vector<int> HungarianAlgorithm::solve(const std::vector<std::vector<double>>& cost_matrix) {
    int n = cost_matrix.size();
    if (n == 0) return {};

    // Create a working copy of the cost matrix
    std::vector<std::vector<double>> cost = cost_matrix;

    // Step 1: Reduce rows and columns
    reduceRows(cost, n);
    reduceCols(cost, n);

    // Step 2: Find optimal assignment using marking algorithm
    std::vector<bool> row_covered(n, false);
    std::vector<bool> col_covered(n, false);
    std::vector<int> assignment(n, -1);  // assignment[i] = j means row i is assigned to column j
    std::vector<std::vector<int>> starred(n, std::vector<int>(n, 0));  // 1 = starred zero, 2 = primed zero

    // Star zeros
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (std::abs(cost[i][j]) < 1e-9 && !row_covered[i] && !col_covered[j]) {
                starred[i][j] = 1;
                row_covered[i] = true;
                col_covered[j] = true;
            }
        }
    }

    // Clear covers
    std::fill(row_covered.begin(), row_covered.end(), false);
    std::fill(col_covered.begin(), col_covered.end(), false);

    // Cover columns with starred zeros
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (starred[i][j] == 1) {
                col_covered[j] = true;
            }
        }
    }

    // Main loop
    int max_iterations = n * n * 10;  // Prevent infinite loops
    int iteration = 0;

    while (iteration++ < max_iterations) {
        // Check if all columns are covered
        int covered_count = 0;
        for (int j = 0; j < n; ++j) {
            if (col_covered[j]) covered_count++;
        }

        if (covered_count >= n) {
            break;  // Optimal assignment found
        }

        // Find an uncovered zero
        int row, col;
        if (findUncoveredZero(cost, n, row_covered, col_covered, row, col)) {
            // Prime this zero
            starred[row][col] = 2;

            // Find if there's a starred zero in this row
            int star_col = -1;
            for (int j = 0; j < n; ++j) {
                if (starred[row][j] == 1) {
                    star_col = j;
                    break;
                }
            }

            if (star_col >= 0) {
                // Cover this row and uncover the column with the starred zero
                row_covered[row] = true;
                col_covered[star_col] = false;
            } else {
                // No starred zero in this row - augment path
                std::vector<std::pair<int, int>> path;
                path.push_back({row, col});

                // Alternate between primed and starred zeros
                bool done = false;
                while (!done) {
                    // Find starred zero in the column
                    int star_row = -1;
                    for (int i = 0; i < n; ++i) {
                        if (starred[i][col] == 1) {
                            star_row = i;
                            break;
                        }
                    }

                    if (star_row < 0) {
                        done = true;
                    } else {
                        path.push_back({star_row, col});

                        // Find primed zero in this row
                        for (int j = 0; j < n; ++j) {
                            if (starred[star_row][j] == 2) {
                                col = j;
                                path.push_back({star_row, j});
                                break;
                            }
                        }
                    }
                }

                // Augment path: unstar all starred zeros, star all primed zeros
                for (size_t i = 0; i < path.size(); ++i) {
                    int r = path[i].first;
                    int c = path[i].second;
                    if (starred[r][c] == 1) {
                        starred[r][c] = 0;
                    } else {
                        starred[r][c] = 1;
                    }
                }

                // Clear all primes
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        if (starred[i][j] == 2) {
                            starred[i][j] = 0;
                        }
                    }
                }

                // Clear covers
                std::fill(row_covered.begin(), row_covered.end(), false);
                std::fill(col_covered.begin(), col_covered.end(), false);

                // Cover columns with starred zeros
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        if (starred[i][j] == 1) {
                            col_covered[j] = true;
                        }
                    }
                }
            }
        } else {
            // No uncovered zero found - adjust cost matrix
            double min_val = findMinUncovered(cost, n, row_covered, col_covered);

            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (row_covered[i]) {
                        cost[i][j] += min_val;
                    }
                    if (!col_covered[j]) {
                        cost[i][j] -= min_val;
                    }
                }
            }
        }
    }

    // Extract assignment from starred zeros
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (starred[i][j] == 1) {
                assignment[i] = j;
                break;
            }
        }
    }

    return assignment;
}

// =============== ADVANCED COST FUNCTIONS ===============

double HungarianAlgorithm::estimateDubinsPathLength(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_heading,
    const Eigen::Vector3d& end_pos,
    double min_turn_radius) {

    // Simplified Dubins path estimation: straight-line distance + turning cost
    Eigen::Vector3d displacement = end_pos - start_pos;
    double straight_dist = displacement.norm();

    if (straight_dist < 1e-6) {
        return 0.0;
    }

    // Normalize vectors
    Eigen::Vector3d desired_direction = displacement / straight_dist;
    Eigen::Vector3d current_heading_norm = start_heading.normalized();

    // Calculate heading change angle
    double cos_angle = current_heading_norm.dot(desired_direction);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));  // Clamp to [-1, 1]
    double heading_change = std::acos(cos_angle);

    // Dubins-like cost: arc length for turning + straight distance
    double turn_arc_length = heading_change * min_turn_radius;

    return turn_arc_length + straight_dist;
}

double HungarianAlgorithm::calculateHeadingChangeCost(
    const Eigen::Vector3d& current_heading,
    const Eigen::Vector3d& target_direction) {

    // Normalize vectors
    Eigen::Vector3d current_norm = current_heading.normalized();
    Eigen::Vector3d target_norm = target_direction.normalized();

    // Calculate angle between headings
    double cos_angle = current_norm.dot(target_norm);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));

    double angle = std::acos(cos_angle);

    // Normalize to [0, 1] range (pi radians = 1.0 cost)
    return angle / M_PI;
}

double HungarianAlgorithm::calculateLineProjectionCost(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& line_direction) {

    // Project positions onto the line direction
    Eigen::Vector3d line_dir_norm = line_direction.normalized();

    double current_projection = current_pos.dot(line_dir_norm);
    double target_projection = target_pos.dot(line_dir_norm);

    // Cost is the difference in projections (order violation)
    double projection_diff = std::abs(current_projection - target_projection);

    return projection_diff;
}

double HungarianAlgorithm::calculatePolygonAngleCost(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& formation_center) {

    // Calculate angles from formation center
    Eigen::Vector3d current_vec = current_pos - formation_center;
    Eigen::Vector3d target_vec = target_pos - formation_center;

    if (current_vec.norm() < 1e-6 || target_vec.norm() < 1e-6) {
        return 0.0;
    }

    double current_angle = std::atan2(current_vec.y(), current_vec.x());
    double target_angle = std::atan2(target_vec.y(), target_vec.x());

    // Angular distance
    double angle_diff = std::abs(current_angle - target_angle);
    if (angle_diff > M_PI) {
        angle_diff = 2.0 * M_PI - angle_diff;
    }

    // Normalize to [0, 1] range
    return angle_diff / M_PI;
}

std::vector<std::vector<double>> HungarianAlgorithm::createAdvancedCostMatrix(
    const std::vector<Eigen::Vector3d>& current_positions,
    const std::vector<Eigen::Vector3d>& current_headings,
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<int>& prev_assignment,
    const AssignmentCostParams& params) {

    int n = current_positions.size();
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n, 0.0));

    // Calculate formation center for polygon cost
    Eigen::Vector3d formation_center = Eigen::Vector3d::Zero();
    for (const auto& pos : target_positions) {
        formation_center += pos;
    }
    formation_center /= static_cast<double>(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double cost = 0.0;

            // 1. Dubins-like distance cost
            Eigen::Vector3d target_direction = target_positions[j] - current_positions[i];
            double dist_cost = estimateDubinsPathLength(
                current_positions[i],
                current_headings[i],
                target_positions[j],
                params.min_turn_radius
            );
            cost += params.w_dist * dist_cost;

            // 2. Heading change cost
            if (target_direction.norm() > 1e-6) {
                double heading_cost = calculateHeadingChangeCost(
                    current_headings[i],
                    target_direction
                );
                cost += params.w_head * heading_cost;
            }

            // 3. Switching penalty (prefer keeping previous assignment)
            if (!prev_assignment.empty() && i < prev_assignment.size()) {
                if (prev_assignment[i] != j) {
                    cost += params.w_switch;
                }
            }

            // 4. Shape-specific cost
            if (params.formation_type.find("line") != std::string::npos) {
                // Line formation: prefer order-preserving assignment
                double line_cost = calculateLineProjectionCost(
                    current_positions[i],
                    target_positions[j],
                    params.formation_direction
                );
                cost += params.w_shape * line_cost;
            } else {
                // Polygon formation: prefer angle-preserving assignment
                double angle_cost = calculatePolygonAngleCost(
                    current_positions[i],
                    target_positions[j],
                    formation_center
                );
                cost += params.w_shape * angle_cost;
            }

            cost_matrix[i][j] = cost;
        }
    }

    return cost_matrix;
}

double HungarianAlgorithm::calculateTotalCost(
    const std::vector<int>& assignment,
    const std::vector<std::vector<double>>& cost_matrix) {

    double total_cost = 0.0;
    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0 && assignment[i] < cost_matrix[i].size()) {
            total_cost += cost_matrix[i][assignment[i]];
        }
    }
    return total_cost;
}

bool HungarianAlgorithm::checkHysteresis(
    double old_cost,
    double new_cost,
    const std::vector<double>& individual_improvements,
    const AssignmentCostParams& params) {

    // Global improvement threshold
    double global_improvement = old_cost - new_cost;
    if (global_improvement < params.tau_global) {
        return false;  // Global improvement not sufficient
    }

    // Individual improvement threshold
    for (double improvement : individual_improvements) {
        if (improvement < params.tau_individual && improvement > -1e-6) {
            // If any drone doesn't improve enough (or gets worse), reject
            return false;
        }
    }

    return true;  // All thresholds satisfied
}

std::vector<int> HungarianAlgorithm::orderPreservingMatch(
    const std::vector<Eigen::Vector3d>& current_positions,
    const std::vector<Eigen::Vector3d>& target_positions,
    const Eigen::Vector3d& line_direction) {

    int n = current_positions.size();

    // FIXED: Instead of using projection-based ordering which causes role swapping
    // when the formation rotates, use simple distance-based Hungarian algorithm.
    // This ensures drones take the nearest available position, maintaining their
    // relative spatial relationships even when the line formation changes direction.

    // Create simple Euclidean distance cost matrix
    auto cost_matrix = createCostMatrix(current_positions, target_positions);

    // Use Hungarian algorithm to find optimal assignment based on distance
    // This naturally preserves spatial relationships without explicitly enforcing order
    return solve(cost_matrix);
}

std::vector<int> HungarianAlgorithm::randomAssignment(int n) {
    // Create identity assignment first
    std::vector<int> assignment(n);
    std::iota(assignment.begin(), assignment.end(), 0);

    // Shuffle randomly using system random device
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(assignment.begin(), assignment.end(), gen);

    return assignment;
}

std::vector<int> HungarianAlgorithm::identityAssignment(int n) {
    // Identity assignment: drone i -> target i
    std::vector<int> assignment(n);
    std::iota(assignment.begin(), assignment.end(), 0);
    return assignment;
}

} // namespace path_manager
