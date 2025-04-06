#ifndef PATH_VISUALIZATION_H
#define PATH_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "px4_msgs/msg/monitoring.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include <thread>
#include "path_planner/grid_map.h"
#include "path_planner/dyn_a_star.h"
#include "path_optimizer/poly_traj_optimizer.h"

class PathVisualization : public rclcpp::Node
{
public:
  PathVisualization();

private:
  visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                               double scale, float r, float g, float b, float alpha);

  void initOptimizer();
  void positionCallback(const px4_msgs::msg::Monitoring::SharedPtr msg);
  void computeAndPublishPaths();
  void publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub);
  void publishObstacles();
  void publishPaths();
  void publishControlPoints(const Eigen::MatrixXd &control_points, int id, float r, float g, float b);
  void getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
                      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                      Eigen::Vector3d &local_target_vel, double &t_to_target);

  void publishIntermediateTraj(const poly_traj::Trajectory &traj, int iteration);
  void publishIntermediateControlPoints(const Eigen::MatrixXd &control_points, int iteration);
  void publishCostText(double cost, int iteration);

  struct GlobalTraj
  {
    poly_traj::Trajectory traj;
    double global_start_time = 0.0;
    double duration = 0.0;
    double glb_t_of_lc_tgt = 0.0;
    double last_glb_t_of_lc_tgt = 0.0;
  };

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr min_jerk_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_traj_pub_;
  rclcpp::Subscription<px4_msgs::msg::Monitoring>::SharedPtr position_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_control_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cost_text_pub_;
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