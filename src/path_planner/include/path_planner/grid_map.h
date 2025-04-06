#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <memory>

using namespace std;

struct MappingParameters
{
  /* map properties */
  Eigen::
      Vector3d map_origin_,
      map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  double resolution_, resolution_inv_;
  double obstacles_inflation_ = 0.2;  // 장애물 인플레이션 거리 (기본값: 10cm)
  double virtual_ceil_height_ = -0.1; // 가상 천장 높이 (기본값: 사용 안 함)
};

struct MappingData
{
  std::vector<double> occupancy_buffer_;          // 정적 Occupancy 데이터
  std::vector<char> occupancy_buffer_inflate_;    // 인플레이트된 Occupancy
  std::vector<char> occupancy_buffer_neg_;        // 음의 Occupancy 버퍼 추가
  std::vector<double> distance_buffer_;           // Positive ESDF
  std::vector<double> distance_buffer_neg_;       // Negative ESDF
  std::vector<double> distance_buffer_all_;       // Combined ESDF
  std::vector<double> tmp_buffer1_, tmp_buffer2_; // ESDF 계산용 임시 버퍼

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  enum
  {
    INVALID_IDX = -10000
  };

  void initMap(const Eigen::Vector3d &map_size, double resolution);
  void setStaticMap(const std::vector<double> &static_occupancy);
  void setOccupancy(const Eigen::Vector3i &id, double occ);
  void inflatePoint(const Eigen::Vector3i &pt, int step);

  void updateESDF3d();
  double getDistance(const Eigen::Vector3d &pos);
  double getDistance(const Eigen::Vector3i &id);

  void evaluateEDT(const Eigen::Vector3d &pos, double &dist);
  void evaluateFirstGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int x, int y, int z);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);
  inline bool isOccupied(const Eigen::Vector3i &id);
  inline void boundIndex(Eigen::Vector3i &id);
  inline int getOccupancy(const Eigen::Vector3d &pos);
  inline int getOccupancy(const Eigen::Vector3i &id);
  inline int getInflateOccupancy(const Eigen::Vector3d &pos);

  inline double getResolution();
  Eigen::Vector3i getVoxelNum() const { return mp_.map_voxel_num_; }

  void getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d &diff, double &value);
  void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d &diff, Eigen::Vector3d &grad);

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
};

inline bool GridMap::isOccupied(const Eigen::Vector3i &id)
{
  if (!isInMap(id))
    return false;
  int idx = toAddress(id);
  return md_.occupancy_buffer_[idx] > 0.5;
}

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int x, int y, int z)
{
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  id(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  return (pos(0) >= mp_.map_min_boundary_(0) + 1e-4 && pos(1) >= mp_.map_min_boundary_(1) + 1e-4 &&
          pos(2) >= mp_.map_min_boundary_(2) + 1e-4 && pos(0) <= mp_.map_max_boundary_(0) - 1e-4 &&
          pos(1) <= mp_.map_max_boundary_(1) - 1e-4 && pos(2) <= mp_.map_max_boundary_(2) - 1e-4);
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  return (idx(0) >= 0 && idx(1) >= 0 && idx(2) >= 0 && idx(0) < mp_.map_voxel_num_(0) &&
          idx(1) < mp_.map_voxel_num_(1) && idx(2) < mp_.map_voxel_num_(2));
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline int GridMap::getOccupancy(const Eigen::Vector3d &pos)
{
  if (!isInMap(pos))
    return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0; // 임계값 0.5
}

inline int GridMap::getOccupancy(const Eigen::Vector3i &id)
{
  if (!isInMap(id))
    return -1;
  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(const Eigen::Vector3d &pos)
{
  if (!isInMap(pos))
    return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline double GridMap::getDistance(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);
  return md_.distance_buffer_all_[toAddress(id)];
}

inline double GridMap::getDistance(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

#endif