// Path shortening with SDF visibility and optional threat avoidance.
// Drops an intermediate waypoint only when the straight connection between
// its neighbours is (a) clear of obstacles in SDF and (b) outside the
// detection range of every supplied threat zone.

#ifndef PATH_PLANNER_SDF_PATH_SHORTENING_H_
#define PATH_PLANNER_SDF_PATH_SHORTENING_H_

#include <Eigen/Core>
#include <vector>

#include "path_planner/sdf/sdf_manager.h"
#include "path_planner/sdf/sdf_query_adapter.h"

namespace path_planner {
namespace sdf {

// True iff the straight [a, b] segment stays clear of SDF obstacles and
// outside every threat detection radius.
inline bool segmentClear(const SDFManager& sdf,
                          const std::vector<ThreatZoneLite>* threats,
                          const Eigen::Vector3d& a,
                          const Eigen::Vector3d& b,
                          double safety_margin,
                          double step_m) {
  double dist = (b - a).norm();
  if (dist <= 1e-6) return true;
  int n = std::max(2, (int)std::ceil(dist / std::max(step_m, 1e-3)));
  for (int i = 0; i <= n; ++i) {
    double t = double(i) / double(n);
    Eigen::Vector3d p = a + t * (b - a);
    float d = sdf.getDistance(p);
    if (!std::isfinite(d) || d < safety_margin) return false;
    if (threats) {
      for (const auto& tz : *threats) {
        if ((p - tz.center).norm() < tz.detection_range) return false;
      }
    }
  }
  return true;
}

// Iteratively drop waypoint i if prev -> next is clear.
inline std::vector<Eigen::Vector3d> shortenPath(
    const SDFManager& sdf,
    const std::vector<Eigen::Vector3d>& path,
    double safety_margin,
    const std::vector<ThreatZoneLite>* threats = nullptr,
    double step_m = -1.0) {
  if (path.size() <= 2) return path;
  if (step_m <= 0.0) step_m = sdf.voxelSize();

  std::vector<Eigen::Vector3d> out = path;
  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<Eigen::Vector3d> next;
    next.reserve(out.size());
    next.push_back(out.front());
    size_t i = 1;
    while (i + 1 < out.size()) {
      const Eigen::Vector3d& prev = next.back();
      const Eigen::Vector3d& nxt = out[i + 1];
      if (segmentClear(sdf, threats, prev, nxt, safety_margin, step_m)) {
        i += 1;
        changed = true;
      } else {
        next.push_back(out[i]);
        i += 1;
      }
    }
    next.push_back(out.back());
    out = std::move(next);
  }
  return out;
}

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_PATH_SHORTENING_H_
