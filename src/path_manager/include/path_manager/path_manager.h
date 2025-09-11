#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/grid_map.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"
#include "path_optimizer/plan_container.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sys/resource.h>
#include <sys/time.h>

using namespace ego_planner;

namespace path_manager
{
  class PathManager
  {
  public:
    PathManager(rclcpp::Node::SharedPtr node);

    void initOptimizer();
    bool isOptimizerInitialized() const { return is_optimizer_initialized_ && poly_traj_opt_ != nullptr; }
    void getLocalTarget(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                        Eigen::Vector3d &local_target_vel, double &t_to_target);
    bool computeAndOptimizePath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                const double trajectory_start_time, const Eigen::Vector3d &local_target_pt,
                                const Eigen::Vector3d &local_target_vel, const bool flag_polyInit, const bool flag_randomPolyTraj,
                                const bool use_formation, const bool have_local_traj);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool checkCollision(int drone_id);

    void deliverTrajToOptimizer(void) { 
        if (!is_optimizer_initialized_ || !poly_traj_opt_) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot deliver trajectory to optimizer - not initialized!");
            return;
        }
        try {
            poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in deliverTrajToOptimizer: %s", e.what());
        }
    };
    void setDroneIdtoOpt(void) { 
        if (!is_optimizer_initialized_ || !poly_traj_opt_) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot set drone ID to optimizer - not initialized!");
            return;
        }
        poly_traj_opt_->setDroneId(0); 
    }
    double getSwarmClearance(void) { 
        if (!is_optimizer_initialized_ || !poly_traj_opt_) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot get swarm clearance - optimizer not initialized!");
            return 2.0; // Default safe clearance
        }
        return poly_traj_opt_->getSwarmClearance(); 
    }
    void setFormationToOptimizer(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size) {
      if (!is_optimizer_initialized_) {
        RCLCPP_WARN(node_->get_logger(), "Optimizer not initialized yet, skipping setFormation");
        return;
      }
      if (!poly_traj_opt_) {
        RCLCPP_ERROR(node_->get_logger(), "poly_traj_opt_ is nullptr despite is_optimizer_initialized_ being true!");
        is_optimizer_initialized_ = false;  // Reset the flag to prevent further attempts
        return;
      }
      
      try {
        poly_traj_opt_->setFormation(formation_positions, formation_size);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in setFormation: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Unknown exception in setFormation");
      }
    }

    TrajContainer traj_;
    
    void updateRobotState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& local_target_pt);

  private:
    bool computeInitReferenceState(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
                                   const Eigen::Vector3d &local_target_vel, const double &ts,
                                   poly_traj::MinJerkOpt &initMJO, const bool flag_polyInit);
    std::shared_ptr<rclcpp::Node> node_;
    GridMap::Ptr grid_map_;
    AStar astar_;
    std::vector<Eigen::Vector3d> simple_path_;
    std::vector<Eigen::Vector3d> obstacle_centers_;
    std::vector<LocalTrajData> swarm_traj_;
    double max_vel_;
    double max_acc_;
    double poly_traj_piece_length_;
    double planning_horizen_;
    ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
    bool is_optimizer_initialized_;
    bool first_call_;
    Eigen::Vector3d current_start_pt_, current_target_pt_;
    bool has_valid_state_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simple_path_pub_;
  };

} // namespace path_manager

#endif // PATH_MANAGER_H