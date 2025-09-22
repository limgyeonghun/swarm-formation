#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/grid_map.h"
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
        
        // Reset first_call_ to true when formation changes
        first_call_ = true;
        RCLCPP_INFO(node_->get_logger(), "Reset first_call_ to true due to formation change");
    }

    TrajContainer traj_;
    
    void updateRobotState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& local_target_pt);
    bool isMapReady(const Eigen::Vector3d& start_pos) const;

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

    std::shared_ptr<swarm_formation::LogManager> log_manager_;
    bool enable_debug_logs_;

  };

} // namespace path_manager

#endif // PATH_MANAGER_H