// SDFManager: CPU ESDF (Felzenszwalb-Huttenlocher + OpenMP) with flat
// binary save/load. ESDF encodes only static obstacles (terrain, buildings).
// SAM zones are handled separately as threat cost in the optimizer.

#ifndef PATH_PLANNER_SDF_MANAGER_H_
#define PATH_PLANNER_SDF_MANAGER_H_

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>

namespace path_planner {
namespace sdf {

struct SDFManagerImpl;  // pimpl

class SDFManager {
 public:
  SDFManager();
  ~SDFManager();

  SDFManager(const SDFManager&) = delete;
  SDFManager& operator=(const SDFManager&) = delete;

  // voxel_size in meters.
  bool initialize(double voxel_size);

  // occupancy layout: ((x * ny) + y) * nz + z (numpy C-order).
  // 0 = free, nonzero = occupied.
  bool buildFromVoxels(const uint8_t* occupancy,
                       int nx, int ny, int nz,
                       const Eigen::Vector3d& origin);

  bool saveToFile(const std::string& path) const;
  bool loadFromFile(const std::string& path,
                    const Eigen::Vector3d& bbox_lo,
                    const Eigen::Vector3d& bbox_hi);

  // Returns signed distance in meters. +inf if outside map or unobserved.
  float getDistance(const Eigen::Vector3d& pos) const;

  // Gradient points away from nearest obstacle.
  bool getDistanceAndGradient(const Eigen::Vector3d& pos,
                              float* distance,
                              Eigen::Vector3d* gradient) const;

  bool isInitialized() const;
  bool hasData() const;
  double voxelSize() const;
  size_t numAllocatedBlocks() const;

  // Grid extent in cells along x/y/z. Zero if no data.
  Eigen::Vector3i shape() const;
  // World-frame origin (lower corner of voxel (0,0,0)).
  Eigen::Vector3d origin() const;

 private:
  std::unique_ptr<SDFManagerImpl> impl_;
};

}  // namespace sdf
}  // namespace path_planner

#endif  // PATH_PLANNER_SDF_MANAGER_H_
