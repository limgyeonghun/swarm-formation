#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/gcopter/sfc_gen.hpp"
#include "path_planner/sdf/sdf_manager.h"
#include "path_planner/sdf/sdf_query_adapter.h"
#include "path_planner/sdf/path_shortening.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"
#include "path_optimizer/plan_container.hpp"
#include "../../common/log_manager.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sys/resource.h>
#include <sys/time.h>
#include <fstream>

using namespace ego_planner;

namespace path_manager
{
  enum class ObstacleShape {
    CIRCLE,
    RECTANGLE
  };

  struct Obstacle {
    Eigen::Vector3d center;
    ObstacleShape shape;
    double param1;  // Circle: radius, Rectangle: width
    double param2;  // Circle: unused, Rectangle: height

    Obstacle() : center(0, 0, 0), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0) {}
    Obstacle(const Eigen::Vector3d& c) : center(c), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0) {}
    Obstacle(const Eigen::Vector3d& c, double radius) : center(c), shape(ObstacleShape::CIRCLE), param1(radius), param2(0.0) {}
    Obstacle(const Eigen::Vector3d& c, double width, double height) : center(c), shape(ObstacleShape::RECTANGLE), param1(width), param2(height) {}
  };

  // Air defense threat zone. Single Gaussian centered at `center` with
  // support out to `detection_range` (sigma = range/3). Peak = max_threat_level.
  struct ThreatZone {
    Eigen::Vector3d center;
    double detection_range;
    double max_threat_level;
  };

  // Terrain elevation data extracted from GridMap
  // Coordinate transform: terrain_publisher uses a different axis convention.
  // terrain → world: X-mirror, then -90° rotation around center.
  // world → terrain: +90° rotation, then X-mirror (inverse).
  struct TerrainData {
    std::vector<float> elevation;  // Column-major elevation data
    int cols = 0;
    int rows = 0;
    double resolution = 0.0;
    double origin_x = 0.0;       // Terrain grid origin X
    double origin_y = 0.0;       // Terrain grid origin Y
    double length_x = 0.0;       // Terrain total length X
    double length_y = 0.0;       // Terrain total length Y
    double center_x = 0.0;       // Terrain center X
    double center_y = 0.0;       // Terrain center Y
    bool valid = false;

    // Convert world (planning) coordinate to terrain grid index and query elevation
    float getElevation(double world_x, double world_y) const {
      if (!valid) return -std::numeric_limits<float>::infinity();

      // Inverse of: terrain → X-mirror → -90° rotate → world
      // Step 1: +90° rotation around terrain center
      double rel_x = world_x - center_x;
      double rel_y = world_y - center_y;
      double rot_x = center_x + rel_y;   // +90°: x' = cy + rel_y
      double rot_y = center_y - rel_x;   // +90°: y' = cy - rel_x

      // Step 2: Undo X-mirror
      double terrain_world_x = 2.0 * center_x - rot_x;
      double terrain_world_y = rot_y;

      // Step 3: World coord → grid index
      int col = static_cast<int>((terrain_world_x - origin_x) / resolution);
      int row = static_cast<int>((terrain_world_y - origin_y) / resolution);

      if (col < 0 || col >= cols || row < 0 || row >= rows) {
        return -std::numeric_limits<float>::infinity();
      }
      int index = col * rows + row;  // Column-major
      if (index < 0 || index >= static_cast<int>(elevation.size())) {
        return -std::numeric_limits<float>::infinity();
      }
      float elev = elevation[index];
      return std::isnan(elev) ? -std::numeric_limits<float>::infinity() : elev;
    }

    // Convert terrain grid cell to world (planning) coordinate (for obstacle_points_)
    Eigen::Vector3d terrainToWorld(int col, int row, float elev) const {
      // Grid index → terrain world coord
      double tw_x = origin_x + (col + 0.5) * resolution;
      double tw_y = origin_y + (row + 0.5) * resolution;

      // X-mirror
      double fx = 2.0 * center_x - tw_x;
      double fy = tw_y;

      // -90° rotation around center
      double rel_x = fx - center_x;
      double rel_y = fy - center_y;
      double wx = center_x - rel_y;
      double wy = center_y + rel_x;

      return Eigen::Vector3d(wx, wy, static_cast<double>(elev));
    }
  };

  class PathManager
  {
  public:
    PathManager(rclcpp::Node::SharedPtr node);

    void initOptimizer(bool force_reinit = false);
    bool isOptimizerInitialized() const { return is_optimizer_initialized_ && poly_traj_opt_ != nullptr; }
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool checkCollision(int drone_id);

    void deliverTrajToOptimizer(void) { 
        if (isOptimizerInitialized()) {
            poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj); 
        }
    };
    void setDroneIdtoOpt(void) { 
        if (isOptimizerInitialized()) {
            poly_traj_opt_->setDroneId(traj_.local_traj.drone_id); 
        }
    }
    double getSwarmClearance(void) { 
        return isOptimizerInitialized() ? poly_traj_opt_->getSwarmClearance() : 0.0; 
    }
    void setFormationToOptimizer(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size) {
        if (!isOptimizerInitialized()) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot set formation: optimizer not initialized!");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Setting formation with %zu positions to optimizer", formation_positions.size());
        poly_traj_opt_->setFormation(formation_positions, formation_size);
    }

    TrajContainer traj_;

    void updateRobotState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& local_target_pt);
    bool isMapReady(const Eigen::Vector3d& start_pos) const;

    // Set formation information for path planning
    void setFormationInfo(int drone_id, const std::string& formation_type,
                         const std::vector<Eigen::Vector3d>& formation_pattern);

    void setLengthPerPiece(double val) { length_per_piece_ = val; }
    void setObstacleClearance(double val) { obstacle_clearance_ = val; }

    // Emergency stop: generate hovering trajectory at current position
    bool EmergencyStop(const Eigen::Vector3d& stop_pos);

    // Terrain data interface
    void setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg);
    bool hasTerrainData() const { return terrain_data_.valid; }

  private:
    // Helper functions for outer/inner line calculation
    std::vector<Eigen::Vector3d> adjustWaypointsForFormation(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& start_pos);

    // Separate logic for line formations (no outer/inner line needed)
    std::vector<Eigen::Vector3d> adjustWaypointsForLineFormation(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& start_pos);

    // Logic for other formations (outer/inner line with curvature)
    std::vector<Eigen::Vector3d> adjustWaypointsWithCurvature(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& start_pos);

    double computePathCurvature(const Eigen::Vector3d& p1,
                               const Eigen::Vector3d& p2,
                               const Eigen::Vector3d& p3);

    Eigen::Vector3d computeLateralOffset(const Eigen::Vector3d& prev_point,
                                        const Eigen::Vector3d& curr_point,
                                        const Eigen::Vector3d& next_point,
                                        double offset_distance);

    std::shared_ptr<rclcpp::Node> node_;
    std::vector<Eigen::Vector3d> simple_path_;
    std::vector<Obstacle> obstacle_centers_;
    std::vector<ThreatZone> threat_zones_;
    double threat_weight_;
    Eigen::Vector3d map_lower_bound_;
    Eigen::Vector3d map_upper_bound_;
    std::vector<LocalTrajData> swarm_traj_;
    double max_vel_;
    double max_acc_;
    double length_per_piece_ = 2.0;
    double obstacle_clearance_ = 0.5;
    TerrainData terrain_data_;

    // ESDF map for SDF-based RRT* queries (phase 3).
    // Built from terrain + obstacle_centers_ inside planGlobalTraj.
    path_planner::sdf::SDFManager sdf_manager_;
    double sdf_voxel_size_ = 1.0;  // m

    // 3D A* front-end. Uses ESDF for collision, threat_zones_ for soft cost.
    path_planner::astar::AStar astar_;
    bool astar_initialized_ = false;
    Eigen::Vector3i astar_pool_size_ = Eigen::Vector3i(120, 120, 40);

    // Precomputed-ESDF paths (both optional, via yaml).
    //   load: if set and file present, skip voxelization on first plan.
    //   save: if set, write the freshly built ESDF after first build.
    // When either is set, the ESDF covers the full loaded terrain (not the
    // per-mission bbox) so the cached map is reusable across missions.
    std::string save_terrain_esdf_path_;
    std::string load_terrain_esdf_path_;
    bool sdf_loaded_from_file_ = false;

    // Full-terrain bbox used when save/load is active. Computed once from
    // terrain_data_ metadata on first use.
    bool terrain_bbox_computed_ = false;
    Eigen::Vector3d terrain_bbox_lo_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d terrain_bbox_hi_ = Eigen::Vector3d::Zero();
    bool computeTerrainBBox(Eigen::Vector3d* lo, Eigen::Vector3d* hi);

    // Rebuilds sdf_manager_ from current terrain + obstacles, covering the
    // bounding box given in world coords. Returns true on success.
    bool buildSDFForBounds(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi);

    ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
    bool is_optimizer_initialized_;
    Eigen::Vector3d current_start_pt_, current_target_pt_;
    bool has_valid_state_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simple_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ctrl_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shorten_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_minco_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr esdf_occ_pub_;

    std::shared_ptr<swarm_formation::LogManager> log_manager_;
    bool enable_debug_logs_;

    // Formation information for path adjustment
    int current_drone_id_;
    std::string current_formation_type_;
    std::vector<Eigen::Vector3d> current_formation_pattern_;

    void setInitialFromPath(const Eigen::Matrix3Xd &path,
                            const double &speed,
                            const Eigen::VectorXi &intervalNs,
                            Eigen::Matrix3Xd &innerPoints,
                            Eigen::VectorXd &timeAlloc);

  };

} // namespace path_manager

#endif // PATH_MANAGER_H