#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/gcopter/sfc_gen.hpp"
#include "path_planner/gcopter/geo_utils.hpp"
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

  // Collision check: geometry obstacles + terrain elevation
  struct ObstacleQueryAdapter {
    const std::vector<Obstacle> *obstacles = nullptr;
    const TerrainData *terrain = nullptr;
    double safety_margin = 0.5;  // Extra clearance around obstacles
    double terrain_clearance = 0.0;  // Min height above terrain

    int query(const Eigen::Vector3d &pos) const {
      // Check geometry obstacles
      if (obstacles) {
        for (const auto &obs : *obstacles) {
          Eigen::Vector2d diff_2d(pos.x() - obs.center.x(), pos.y() - obs.center.y());
          double dist_2d = diff_2d.norm();

          if (obs.shape == ObstacleShape::CIRCLE) {
            double radius = (obs.param1 > 0) ? obs.param1 : 0.5;
            if (dist_2d < radius + safety_margin) return 1;
          } else if (obs.shape == ObstacleShape::RECTANGLE) {
            double half_w = obs.param1 / 2.0 + safety_margin;
            double half_h = obs.param2 / 2.0 + safety_margin;
            if (std::abs(pos.x() - obs.center.x()) < half_w &&
                std::abs(pos.y() - obs.center.y()) < half_h) return 1;
          }
        }
      }
      // Check terrain collision
      if (terrain && terrain->valid) {
        float elev = terrain->getElevation(pos.x(), pos.y());
        if (elev > -1e10 && pos.z() < elev + terrain_clearance) return 1;
      }
      return 0;
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
    void setTerrainClearance(double val) { terrain_clearance_ = val; }

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

    // SFC corridor data
    std::vector<Eigen::MatrixX4d> global_hpolys_;     // Global SFC corridor (H-polytopes)
    std::vector<Eigen::Vector3d> obstacle_points_;      // Obstacle point cloud for SFC generation
    Eigen::Vector3d map_lower_bound_;                   // Map bounds
    Eigen::Vector3d map_upper_bound_;
    std::vector<LocalTrajData> swarm_traj_;
    double max_vel_;
    double max_acc_;
    double length_per_piece_ = 2.0;
    double obstacle_clearance_ = 0.5;
    double sfc_progress_;
    double sfc_range_;
    double z_min_;
    double terrain_clearance_;
    double terrain_sample_spacing_;
    TerrainData terrain_data_;
    ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
    bool is_optimizer_initialized_;
    Eigen::Vector3d current_start_pt_, current_target_pt_;
    bool has_valid_state_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simple_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sfc_corridor_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shortest_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_points_pub_;

    std::shared_ptr<swarm_formation::LogManager> log_manager_;
    bool enable_debug_logs_;

    // Formation information for path adjustment
    int current_drone_id_;
    std::string current_formation_type_;
    std::vector<Eigen::Vector3d> current_formation_pattern_;


    // GCOPTER-style shortest path through corridor overlaps
    typedef Eigen::Matrix3Xd PolyhedronV;
    typedef Eigen::MatrixX4d PolyhedronH;
    typedef std::vector<PolyhedronV> PolyhedraV;
    typedef std::vector<PolyhedronH> PolyhedraH;

    void publishSFCCorridor(const PolyhedraH &hPolys);
    void publishShortestPath(const Eigen::Matrix3Xd &path);

    bool processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs);

    static double costDistance(void *ptr,
                               const Eigen::VectorXd &xi,
                               Eigen::VectorXd &gradXi);

    void getShortestPath(const Eigen::Vector3d &ini,
                         const Eigen::Vector3d &fin,
                         const PolyhedraV &vPolys,
                         const double &smoothD,
                         Eigen::Matrix3Xd &path);

    void setInitialFromPath(const Eigen::Matrix3Xd &path,
                            const double &speed,
                            const Eigen::VectorXi &intervalNs,
                            Eigen::Matrix3Xd &innerPoints,
                            Eigen::VectorXd &timeAlloc);

  };

} // namespace path_manager

#endif // PATH_MANAGER_H