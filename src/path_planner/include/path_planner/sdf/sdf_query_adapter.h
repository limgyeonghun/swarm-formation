// RRT* query adapter backed by SDFManager.
// Replaces ObstacleQueryAdapter: instead of per-query obstacle loop,
// uses precomputed ESDF for O(1) collision check.
// Threat zones are kept separate (not baked into SDF).

#ifndef PATH_PLANNER_SDF_QUERY_ADAPTER_H_
#define PATH_PLANNER_SDF_QUERY_ADAPTER_H_

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "path_planner/sdf/sdf_manager.h"

namespace path_planner {
namespace sdf {

struct ThreatZoneLite {
  Eigen::Vector3d center;
  double detection_range;
  double max_threat_level;
};

struct SDFQueryAdapter {
  const SDFManager *sdf = nullptr;
  const std::vector<ThreatZoneLite> *threat_zones = nullptr;
  double safety_margin = 0.5;
  double threat_weight = 10.0;

  // Hard collision: 1 if obstacle (SDF <= safety_margin), 0 if free.
  int query(const Eigen::Vector3d &pos) const {
    if (!sdf || !sdf->hasData()) return 0;
    float d = sdf->getDistance(pos);
    if (!std::isfinite(d)) return 1;  // outside map == blocked
    return (d < safety_margin) ? 1 : 0;
  }

  double getThreatLevel(const Eigen::Vector3d &pos) const {
    if (!threat_zones || threat_zones->empty()) return 0.0;
    double total = 0.0;
    for (const auto &tz : *threat_zones) {
      double dist = (pos - tz.center).norm();
      if (dist < tz.detection_range) {
        double sigma = tz.detection_range / 3.0;
        total += tz.max_threat_level *
                 std::exp(-0.5 * (dist / sigma) * (dist / sigma));
      }
    }
    return total;
  }

  double getThreatCostMultiplier(const Eigen::Vector3d &pos) const {
    return 1.0 + threat_weight * getThreatLevel(pos);
  }
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_QUERY_ADAPTER_H_
