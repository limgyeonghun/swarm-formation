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
#include <geometry_msgs/msg/point_stamped.hpp>

class PathManager : public rclcpp::Node
{
public:
  PathManager();

private:
  void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void initOptimizer();
  void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg, const poly_traj::Trajectory &traj);
  void getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
                      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                      Eigen::Vector3d &local_target_vel, double &t_to_target);
  void computeAndPublishPaths();

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time;
    double duration;
    double glb_t_of_lc_tgt;
    double last_glb_t_of_lc_tgt;
  } global_traj_;

  struct LocalTrajData
  {
    int drone_id;
    int traj_id;
    double duration;
    double start_time;
    Eigen::Vector3d start_pos; // 추가
    poly_traj::Trajectory traj;
  } local_traj_;

  rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  GridMap::Ptr grid_map_;
  AStar astar_;
  std::vector<Eigen::Vector3d> simple_path_;
  std::vector<Eigen::Vector3d> obstacle_centers_;
  double max_vel_ = 1.0;
  double max_acc_ = 1.0;
  Eigen::Vector3d start_pt_;
  Eigen::Vector3d end_pt_;

  ego_planner::PolyTrajOptimizer::Ptr poly_traj_opt_;
  bool is_optimizer_initialized_;
  bool first_call_;
  Eigen::MatrixXd iniState_;
  Eigen::MatrixXd finState_;
  double current_time_;    // 현재 시간 (초 단위)
  double last_start_time_; // 이전 경로 시작 시간
};

#endif