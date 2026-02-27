#ifndef HUNGARIAN_ALGORITHM_H
#define HUNGARIAN_ALGORITHM_H

#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <string>

namespace path_manager {

/**
 * @brief Cost function parameters for advanced assignment
 */
struct AssignmentCostParams {
    double w_dist = 1.0;           // Weight for distance cost
    double w_head = 0.5;           // Weight for heading change cost
    double w_switch = 2.0;         // Weight for switching penalty (prefer keeping current assignment)
    double w_shape = 0.3;          // Weight for shape-specific cost (line: projection, polygon: angle)
    double min_turn_radius = 1.0;  // Minimum turning radius for Dubins-like path cost
    double tau_individual = 0.1;   // Individual improvement threshold (hysteresis)
    double tau_global = 0.5;       // Global improvement threshold (hysteresis)

    std::string formation_type = "";  // Formation type for shape-specific costs
    Eigen::Vector3d formation_direction{1.0, 0.0, 0.0};  // Direction vector for line formations
};

/**
 * @brief Hungarian Algorithm for optimal assignment problem with advanced cost functions
 *
 * Solves the assignment problem to minimize total cost with:
 * - Dubins-like path cost (considering turning radius)
 * - Heading change penalty
 * - Switching penalty (hysteresis to maintain stable assignments)
 * - Shape-specific costs (line formations: projection-based, polygons: angle-based)
 */
class HungarianAlgorithm {
public:
    /**
     * @brief Solve the assignment problem
     *
     * @param cost_matrix Cost matrix where cost_matrix[i][j] is the cost of assigning worker i to task j
     * @return std::vector<int> Assignment vector where assignment[i] = j means worker i is assigned to task j
     */
    static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix);

    /**
     * @brief Create advanced cost matrix with multiple cost components
     *
     * @param current_positions Current positions of all drones
     * @param current_headings Current heading directions of all drones
     * @param target_positions Target positions in the new formation
     * @param prev_assignment Previous assignment (for switching penalty)
     * @param params Cost function parameters
     * @return std::vector<std::vector<double>> Advanced cost matrix
     */
    static std::vector<std::vector<double>> createAdvancedCostMatrix(
        const std::vector<Eigen::Vector3d>& current_positions,
        const std::vector<Eigen::Vector3d>& current_headings,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<int>& prev_assignment,
        const AssignmentCostParams& params);

    /**
     * @brief Create cost matrix from current positions and target positions (simple version)
     *
     * @param current_positions Current positions of all drones
     * @param target_positions Target positions in the new formation
     * @return std::vector<std::vector<double>> Cost matrix based on Euclidean distances
     */
    static std::vector<std::vector<double>> createCostMatrix(
        const std::vector<Eigen::Vector3d>& current_positions,
        const std::vector<Eigen::Vector3d>& target_positions);

    /**
     * @brief Check if new assignment satisfies hysteresis thresholds
     *
     * @param old_cost Total cost of old assignment
     * @param new_cost Total cost of new assignment
     * @param individual_improvements Per-drone cost improvements
     * @param params Cost function parameters (tau_individual, tau_global)
     * @return bool True if new assignment should be accepted
     */
    static bool checkHysteresis(
        double old_cost,
        double new_cost,
        const std::vector<double>& individual_improvements,
        const AssignmentCostParams& params);

    /**
     * @brief Apply order-preserving matching for line formations
     *
     * @param current_positions Current positions of all drones
     * @param target_positions Target positions in line formation
     * @param line_direction Direction vector of the line
     * @return std::vector<int> Order-preserving assignment
     */
    static std::vector<int> orderPreservingMatch(
        const std::vector<Eigen::Vector3d>& current_positions,
        const std::vector<Eigen::Vector3d>& target_positions,
        const Eigen::Vector3d& line_direction);

    /**
     * @brief Generate random assignment for ablation study
     *
     * @param n Number of drones
     * @return std::vector<int> Random permutation assignment
     */
    static std::vector<int> randomAssignment(int n);

    /**
     * @brief Generate identity assignment (i -> i) for ablation study
     *
     * @param n Number of drones
     * @return std::vector<int> Identity assignment [0,1,2,...]
     */
    static std::vector<int> identityAssignment(int n);

    /**
     * @brief Apply assignment to reorder target positions
     *
     * @param assignment Assignment vector from solve()
     * @param target_positions Original target positions
     * @return std::vector<Eigen::Vector3d> Reordered target positions according to assignment
     */
    static std::vector<Eigen::Vector3d> applyAssignment(
        const std::vector<int>& assignment,
        const std::vector<Eigen::Vector3d>& target_positions);

    /**
     * @brief Calculate total cost for a given assignment
     *
     * @param assignment Assignment vector
     * @param cost_matrix Cost matrix
     * @return double Total cost
     */
    static double calculateTotalCost(
        const std::vector<int>& assignment,
        const std::vector<std::vector<double>>& cost_matrix);

private:
    static const double INF;

    /**
     * @brief Reduce rows of the cost matrix
     */
    static void reduceRows(std::vector<std::vector<double>>& cost, int n);

    /**
     * @brief Reduce columns of the cost matrix
     */
    static void reduceCols(std::vector<std::vector<double>>& cost, int n);

    /**
     * @brief Find uncovered zero in the cost matrix
     */
    static bool findUncoveredZero(const std::vector<std::vector<double>>& cost, int n,
                                   const std::vector<bool>& row_covered,
                                   const std::vector<bool>& col_covered,
                                   int& row, int& col);

    /**
     * @brief Find minimum uncovered value in the cost matrix
     */
    static double findMinUncovered(const std::vector<std::vector<double>>& cost, int n,
                                    const std::vector<bool>& row_covered,
                                    const std::vector<bool>& col_covered);

    /**
     * @brief Estimate Dubins-like path length
     */
    static double estimateDubinsPathLength(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& start_heading,
        const Eigen::Vector3d& end_pos,
        double min_turn_radius);

    /**
     * @brief Calculate heading change cost
     */
    static double calculateHeadingChangeCost(
        const Eigen::Vector3d& current_heading,
        const Eigen::Vector3d& target_direction);

    /**
     * @brief Calculate shape-specific cost for line formations
     */
    static double calculateLineProjectionCost(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& target_pos,
        const Eigen::Vector3d& line_direction);

    /**
     * @brief Calculate shape-specific cost for polygon formations
     */
    static double calculatePolygonAngleCost(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& target_pos,
        const Eigen::Vector3d& formation_center);
};

} // namespace path_manager

#endif // HUNGARIAN_ALGORITHM_H
