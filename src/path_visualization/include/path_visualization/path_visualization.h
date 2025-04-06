#ifndef PATH_VISUALIZATION_H
#define PATH_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
  visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                               double scale, float r, float g, float b, float a);
  void publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub);
  void publishObstacles();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_traj_pub_;
  rclcpp::Subscription<px4_msgs::msg::Monitoring>::SharedPtr position_sub_;
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr optimized_path_sub_;

  Eigen::Vector3d start_pt_;
  std::vector<Eigen::Vector3d> obstacle_centers_;
};

#endif