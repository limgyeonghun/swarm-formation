#ifndef FORMATION_UTILS_H
#define FORMATION_UTILS_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace path_manager {

/**
 * @brief Utility functions for formation pattern generation
 */
class FormationUtils {
public:
    /**
     * @brief Generate formation pattern based on formation type
     *
     * @param formation_type Type of formation (line_first, line_second, square, triangle, etc.)
     * @param num_drones Number of drones in the formation
     * @param scale Scale of the formation
     * @return std::vector<Eigen::Vector3d> Formation pattern as offset vectors from formation center
     */
    static std::vector<Eigen::Vector3d> generateFormationPattern(
        const std::string& formation_type,
        int num_drones,
        double scale);
};

} // namespace path_manager

#endif // FORMATION_UTILS_H
