// RRT* query adapter backed by SDFManager.
// Replaces ObstacleQueryAdapter: instead of per-query obstacle loop,
// uses precomputed ESDF for O(1) collision check.
// Risk zones are kept separate (not baked into SDF).

#ifndef PATH_PLANNER_SDF_QUERY_ADAPTER_H_
#define PATH_PLANNER_SDF_QUERY_ADAPTER_H_

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "path_planner/sdf/sdf_manager.h"

namespace path_planner {
namespace sdf {

struct RiskZoneLite {
  Eigen::Vector3d center;
  double reach;   // meters; risk is exactly zero outside this ball
  double peak;    // dimensionless in (0, 1]
};

struct SDFQueryAdapter {
  const SDFManager *sdf = nullptr;
  const std::vector<RiskZoneLite> *risk_zones = nullptr;
  double safety_margin = 0.5;
  double risk_alpha = 1.0;

  // Hard collision: 1 if obstacle (SDF <= safety_margin), 0 if free.
  int query(const Eigen::Vector3d &pos) const {
    if (!sdf || !sdf->hasData()) return 0;
    float d = sdf->getDistance(pos);
    if (!std::isfinite(d)) return 1;  // outside map == blocked
    return (d < safety_margin) ? 1 : 0;
  }

  double getRiskLevel(const Eigen::Vector3d &pos) const {
    if (!risk_zones || risk_zones->empty()) return 0.0;
    double total = 0.0;
    for (const auto &tz : *risk_zones) {
      double dist = (pos - tz.center).norm();
      if (dist < tz.reach) {
        double sigma = tz.reach / 3.0;
        total += tz.peak *
                 std::exp(-0.5 * (dist / sigma) * (dist / sigma));
      }
    }
    return total;
  }

  double getRiskCostMultiplier(const Eigen::Vector3d &pos) const {
    return 1.0 + risk_alpha * getRiskLevel(pos);
  }
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_QUERY_ADAPTER_H_
