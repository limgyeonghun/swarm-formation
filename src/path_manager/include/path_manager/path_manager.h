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

using namespace ego_planner;

namespace path_manager
{

  class PathManager
  {
  public:
    PathManager(const std::shared_ptr<rclcpp::Node> &node);

    void initOptimizer();
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

    void deliverTrajToOptimizer(void) { poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj); };

    void setDroneIdtoOpt(void) { poly_traj_opt_->setDroneId(0); }

    double getSwarmClearance(void) { return poly_traj_opt_->getSwarmClearance(); }

    TrajContainer traj_;

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
  };

} // namespace path_manager

#endif // PATH_MANAGER_H