#ifndef PATH_VISUALIZATION_H
#define PATH_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Eigen>
#include "path_manager/msg/poly_traj.hpp"
#include "path_optimizer/poly_traj_utils.hpp"

class PathVisualization : public rclcpp::Node
{
public:
  PathVisualization();

private:
  struct DroneData {
    double current_time;
    Eigen::Vector3d start_pt;
    path_manager::msg::PolyTraj current_traj;
  };

  void optimizedPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
  void globalPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
  void updatePosition();
  void logPositions(); // 로그 출력 함수 추가
  visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                               double scale, float r, float g, float b, float a);
  void publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub);
  void publishObstacles();

  int num_drones_;
  std::vector<DroneData> drone_data_;
  std::vector<Eigen::Vector3d> obstacle_centers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_traj_pub_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> position_marker_pubs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> position_pubs_;
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr optimized_path_sub_;
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_; // 업데이트 타이머
  rclcpp::TimerBase::SharedPtr log_timer_; // 로그 타이머 추가
};

#endif