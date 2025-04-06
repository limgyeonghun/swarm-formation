#ifndef PATH_VISUALIZATION_H
#define PATH_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/monitoring.hpp>
#include <Eigen/Eigen>
#include "path_manager/msg/poly_traj.hpp"

class PathVisualization : public rclcpp::Node
{
public:
  PathVisualization();

private:
  void positionCallback(const px4_msgs::msg::Monitoring::SharedPtr msg);
  void optimizedPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
  void updatePosition();
  visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                               double scale, float r, float g, float b, float a);
  void publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub);
  void publishObstacles();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr position_marker_pub_;  // 점 퍼블리셔
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;  // 위치 퍼블리셔
  rclcpp::Subscription<px4_msgs::msg::Monitoring>::SharedPtr position_sub_;
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr optimized_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;  // 0.01초 타이머

  Eigen::Vector3d start_pt_;
  std::vector<Eigen::Vector3d> obstacle_centers_;
  path_manager::msg::PolyTraj current_traj_;  // 현재 로컬 경로
  double current_time_;  // 현재 시간
};

#endif