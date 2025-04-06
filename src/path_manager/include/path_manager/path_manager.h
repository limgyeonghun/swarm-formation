#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "path_planner/grid_map.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include "path_manager/msg/poly_traj.hpp"

class PathManager : public rclcpp::Node
{
public:
  PathManager();
  void computeAndPublishPaths();

private:
  void initOptimizer();
  void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg, const poly_traj::Trajectory &traj);
  void getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
                      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                      Eigen::Vector3d &local_target_vel, double &t_to_target);

  struct GlobalTraj
  {
    poly_traj::Trajectory traj;
    double global_start_time = 0.0;
    double duration = 0.0;
    double glb_t_of_lc_tgt = 0.0;
    double last_glb_t_of_lc_tgt = 0.0;
  };

  rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  GridMap::Ptr grid_map_;
  AStar astar_;
  std::vector<Eigen::Vector3d> simple_path_;
  std::vector<Eigen::Vector3d> obstacle_centers_;
  double max_vel_ = 1.0;
  double max_acc_ = 1.0;
  Eigen::Vector3d start_pt_;
  Eigen::Vector3d end_pt_;
  GlobalTraj global_traj_;
  ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
  bool is_optimizer_initialized_;
  bool first_call_;
  Eigen::MatrixXd iniState_;
  Eigen::MatrixXd finState_;
};

#endif