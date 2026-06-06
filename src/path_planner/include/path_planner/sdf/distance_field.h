// IDistanceField: abstract distance-field query interface.
//
// Decouples the front-end (AStar/FM2) and back-end (PolyTrajOptimizer) from the
// concrete SDFManager so an alternative distance-field provider can be injected.
// These are the only three methods both consumers use.

#ifndef PATH_PLANNER_SDF_DISTANCE_FIELD_H_
#define PATH_PLANNER_SDF_DISTANCE_FIELD_H_

#include <Eigen/Core>

namespace path_planner {
namespace sdf {

class IDistanceField {
 public:
  virtual ~IDistanceField() = default;

  virtual float getDistance(const Eigen::Vector3d& pos) const = 0;

  virtual bool getDistanceAndGradient(const Eigen::Vector3d& pos,
                                      float* distance,
                                      Eigen::Vector3d* gradient) const = 0;

  virtual bool hasData() const = 0;
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_DISTANCE_FIELD_H_
