#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

using namespace std;

struct RoadSegment {
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double width;
};

// Threat zone structure for air defense systems (SAM, AAA, etc.)
struct ThreatZone {
  Eigen::Vector3d center;           // Center position of threat zone
  double detection_range;            // Detection radius (m)
  double engagement_range;           // Engagement/kill radius (m)
  double max_threat_level;           // Maximum threat level (0-100)
  std::string name;                  // Zone name for debugging

  // Optional: elliptical range (different horizontal/vertical)
  bool use_elliptical = false;
  double horizontal_range;
  double vertical_range;
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
  double ground_height_ = 0.0;
  
  // ESDF parameters
  double p_hit_ = 0.70, p_miss_ = 0.35, p_min_ = 0.12, p_max_ = 0.97, p_occ_ = 0.80;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  double unknown_flag_ = 0.01;
  
  // Road boundary parameters for rover operation
  bool use_road_boundary_ = false;
  std::vector<RoadSegment> road_segments_;
  double road_width_ = 8.0;
  double road_margin_ = 0.5;

  // Threat zone parameters for air defense penetration
  bool use_threat_zones_ = false;
  std::vector<ThreatZone> threat_zones_;

  // Terrain gridmap parameters
  bool use_terrain_obstacles_ = false;
  double terrain_obstacle_threshold_ = 0.0;  // Elevation threshold (m) for obstacle
  double terrain_target_cell_size_ = 50.0;   // TARGET_CELL_SIZE_M from terrain_publisher (m)

  // Visualization parameters
  bool publish_global_esdf_viz_ = false;  // Publish global ESDF visualization (debugging)
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

  // Threat field buffer (continuous threat level at each voxel)
  std::vector<double> threat_buffer_;

  // Local bound for ESDF updates
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  bool local_updated_ = false;

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
  void inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts);

  void updateESDF3d();
  void updateESDF3d(const Eigen::Vector3i& min_esdf, const Eigen::Vector3i& max_esdf);
  void updateESDFLocal(const Eigen::Vector3d& center_pos);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);

  // Global ESDF visualization
  void publishGlobalESDFVisualization();

  void evaluateEDT(const Eigen::Vector3d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

  // Threat zone management
  void addThreatZone(const ThreatZone& zone);
  void clearThreatZones();
  void updateThreatField();
  double getThreatLevel(const Eigen::Vector3d& pos) const;
  Eigen::Vector3d getThreatGradient(const Eigen::Vector3d& pos) const;
  const std::vector<ThreatZone>& getThreatZones() const { return mp_.threat_zones_; }

  // Terrain gridmap management
  void processTerrainGridMap(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int x, int y, int z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);
  inline bool isOccupied(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);
  inline bool isInRoadBoundary(const Eigen::Vector3d& pos);
  inline void boundIndex(Eigen::Vector3i& id);
  inline int getOccupancy(const Eigen::Vector3d& pos);
  inline int getOccupancy(const Eigen::Vector3i& id);
  inline int getInflateOccupancy(const Eigen::Vector3d& pos);
  inline int getInflateOccupancy2D(const Eigen::Vector3d& pos);  // 2D optimized version
  inline double getResolution();
  inline double getObstaclesInflation();

  Eigen::Vector3i getVoxelNum() const { return mp_.map_voxel_num_; }
  Eigen::Vector3d getMapMinBoundary() const { return mp_.map_min_boundary_; }
  Eigen::Vector3d getMapMaxBoundary() const { return mp_.map_max_boundary_; }

  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);
  void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);
  
  void clearAndInflateLocalMap();
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  Eigen::Vector3d getOrigin() const { return mp_.map_origin_; }
  Eigen::Vector3d getMapSize() const { return mp_.map_size_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<double> distance_buffer_local_;
  Eigen::Vector3i local_esdf_min_, local_esdf_max_;

  // Terrain gridmap subscriber
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;

  // Global ESDF visualization publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_esdf_pub_;

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

  Eigen::Vector3i id;
  posToIndex(pos, id);
  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;

  return md_.occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
  if (!isInMap(pos)) return -1;

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

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);
  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline int GridMap::getInflateOccupancy2D(const Eigen::Vector3d& pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  for (int z = 0; z < mp_.map_voxel_num_(2); z++) {
    if (md_.occupancy_buffer_inflate_[toAddress(id(0), id(1), z)] == 1) {
      return 1;
    }
  }
  return 0;
}

inline bool GridMap::isInRoadBoundary(const Eigen::Vector3d& pos) {
  if (!mp_.use_road_boundary_) return true;
  if (mp_.road_segments_.empty()) return true;

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

inline double GridMap::getObstaclesInflation() {
  return mp_.obstacles_inflation_;
}
#endif