#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>

using namespace std;

struct RoadSegment {
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double width;
};

struct MappingParameters {
  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_ = 0.1;
  double virtual_ceil_height_ = -0.1;
  int local_map_margin_ = 1;
  std::string frame_id_ = "world";
  double esdf_slice_height_ = -0.1;
  bool show_esdf_time_ = false;
  double local_bound_inflate_ = 1.0;
  
  // ESDF update optimization parameters
  int esdf_update_skip_ = 1;  // Number of frames to skip ESDF update
  double esdf_update_threshold_ = 0.1;  // ESDF update threshold
  
  // Road boundary parameters for rover operation
  bool use_road_boundary_ = false;
  std::vector<RoadSegment> road_segments_;
  double road_width_ = 8.0;
  double road_margin_ = 0.5;
};

struct MappingData {
  Eigen::Vector3d camera_pos_;
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<char> occupancy_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;
  std::vector<double> tmp_buffer1_, tmp_buffer2_;
  bool esdf_need_update_ = false;
  double esdf_time_ = 0.0;
  double max_esdf_time_ = 0.0;
  int update_num_ = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  void initMap(const std::shared_ptr<rclcpp::Node>& node);
  void setStaticMap(const std::vector<double>& static_occupancy);
  void setOccupancy(const Eigen::Vector3i& id, double occ);
  void inflatePoint(const Eigen::Vector3i& pt, int step);

  void updateESDF3d();
  void updateESDF3d(const Eigen::Vector3i& min_esdf, const Eigen::Vector3i& max_esdf);
  void updateESDFLocal(const Eigen::Vector3d& center_pos);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);

  void evaluateEDT(const Eigen::Vector3d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int x, int y, int z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);
  inline bool isOccupied(const Eigen::Vector3i& id);
  inline bool isInRoadBoundary(const Eigen::Vector3d& pos);
  inline void boundIndex(Eigen::Vector3i& id);
  inline int getOccupancy(const Eigen::Vector3d& pos);
  inline int getOccupancy(const Eigen::Vector3i& id);
  inline int getInflateOccupancy(const Eigen::Vector3d& pos);
  inline double getResolution();

  Eigen::Vector3i getVoxelNum() const { return mp_.map_voxel_num_; }
  Eigen::Vector3d getMapMinBoundary() const { return mp_.map_min_boundary_; }
  Eigen::Vector3d getMapMaxBoundary() const { return mp_.map_max_boundary_; }

  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);
  void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<double> distance_buffer_local_;
  Eigen::Vector3i local_esdf_min_, local_esdf_max_;

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
};

inline bool GridMap::isOccupied(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return false;
  int idx = toAddress(id);
  return md_.occupancy_buffer_[idx] > 0.5;
}

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int x, int y, int z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  id(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  return (pos(0) >= mp_.map_min_boundary_(0) + 1e-4 && pos(1) >= mp_.map_min_boundary_(1) + 1e-4 &&
          pos(2) >= mp_.map_min_boundary_(2) + 1e-4 && pos(0) <= mp_.map_max_boundary_(0) - 1e-4 &&
          pos(1) <= mp_.map_max_boundary_(1) - 1e-4 && pos(2) <= mp_.map_max_boundary_(2) - 1e-4);
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  return (idx(0) >= 0 && idx(1) >= 0 && idx(2) >= 0 && idx(0) < mp_.map_voxel_num_(0) &&
          idx(1) < mp_.map_voxel_num_(1) && idx(2) < mp_.map_voxel_num_(2));
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline int GridMap::getOccupancy(const Eigen::Vector3d& pos) {
  if (!isInMap(pos)) return -1;
  
  // 도로 경계 체크: 도로 밖이면 장애물로 처리
  if (!isInRoadBoundary(pos)) return 1;
  
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  
  // 인덱스를 위치로 변환해서 도로 경계 체크
  Eigen::Vector3d pos;
  indexToPos(id, pos);
  if (!isInRoadBoundary(pos)) return 1;
  
  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
  if (!isInMap(pos)) return -1;
  
  // 도로 경계 체크: 도로 밖이면 장애물로 처리
  if (!isInRoadBoundary(pos)) return 1;
  
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline double GridMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);
  return md_.distance_buffer_all_[toAddress(id)];
}

inline double GridMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

inline bool GridMap::isInRoadBoundary(const Eigen::Vector3d& pos) {
  if (!mp_.use_road_boundary_) return true;
  if (mp_.road_segments_.empty()) return false;

  for (const auto& segment : mp_.road_segments_) {

    Eigen::Vector2d road_dir(segment.end_x - segment.start_x, 
                            segment.end_y - segment.start_y);
    double road_length = road_dir.norm();
    if (road_length < 1e-6) continue;
    
    road_dir /= road_length;

    Eigen::Vector2d road_normal(-road_dir.y(), road_dir.x());

    Eigen::Vector2d pos_vec(pos(0) - segment.start_x, 
                           pos(1) - segment.start_y);

    double along_road = pos_vec.dot(road_dir);

    double from_center = std::abs(pos_vec.dot(road_normal));
    
    if (along_road >= -mp_.road_margin_ && 
        along_road <= road_length + mp_.road_margin_ &&
        from_center <= (segment.width / 2.0 - mp_.road_margin_)) {
      return true;
    }
  }
  
  return false;
}

#endif