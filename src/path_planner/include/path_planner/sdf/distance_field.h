// IDistanceField: abstract distance-field query interface.
//
// Decouples the front-end (PathSearcher: A*/FM2) and back-end (PolyTrajOptimizer) from the
// concrete SDFManager so an alternative distance-field provider can be injected.
// These are the only three methods both consumers use.

#ifndef PATH_PLANNER_SDF_DISTANCE_FIELD_H_
#define PATH_PLANNER_SDF_DISTANCE_FIELD_H_

#include <Eigen/Core>
#include <limits>
#include <cstdint>

namespace path_planner {
namespace sdf {

class IDistanceField {
 public:
  virtual ~IDistanceField() = default;

  virtual float getDistance(const Eigen::Vector3d& pos) const = 0;

  // Distance to the DYNAMIC obstacle layer only (RViz-spawned cars/buildings
  // etc.), +inf when no patch covers pos. Lets the planner keep a more
  // generous berth around dynamic obstacles than around terrain. Default
  // +inf so providers without a dynamic layer are unaffected.
  virtual float getDynamicDistance(const Eigen::Vector3d& /*pos*/) const {
    return std::numeric_limits<float>::infinity();
  }

  virtual bool getDistanceAndGradient(const Eigen::Vector3d& pos,
                                      float* distance,
                                      Eigen::Vector3d* gradient) const = 0;

  virtual bool hasData() const = 0;

  // Monotonic change counter: bumps whenever the field's content changes
  // (rebuild, file load, dynamic obstacle add/remove/clear). Lets consumers
  // cache derived products (e.g. the FM2 arrival-time field) and detect
  // staleness cheaply. Providers that never change may return 0.
  virtual uint64_t revision() const { return 0; }
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_DISTANCE_FIELD_H_
