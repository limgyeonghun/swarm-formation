#ifndef PATH_VISUALIZATION_H
#define PATH_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <path_manager/msg/poly_traj.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

enum class ObstacleShape {
  CIRCLE,
  RECTANGLE
};

struct Obstacle {
  Eigen::Vector3d center;
  ObstacleShape shape;
  double param1;
  double param2;
  double z_extent;  // vertical height above center.z; 0 = infinite column (back-compat)

  Obstacle() : center(0, 0, 0), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0), z_extent(0.0) {}
  Obstacle(const Eigen::Vector3d& c) : center(c), shape(ObstacleShape::CIRCLE), param1(-1.0), param2(0.0), z_extent(0.0) {}
  Obstacle(const Eigen::Vector3d& c, double radius) : center(c), shape(ObstacleShape::CIRCLE), param1(radius), param2(0.0), z_extent(0.0) {}
  Obstacle(const Eigen::Vector3d& c, double radius, double height, bool /*circle_with_height*/) : center(c), shape(ObstacleShape::CIRCLE), param1(radius), param2(0.0), z_extent(height) {}
  Obstacle(const Eigen::Vector3d& c, double width, double length) : center(c), shape(ObstacleShape::RECTANGLE), param1(width), param2(length), z_extent(0.0) {}
  Obstacle(const Eigen::Vector3d& c, double width, double length, double height) : center(c), shape(ObstacleShape::RECTANGLE), param1(width), param2(length), z_extent(height) {}
};

struct VisRiskZone {
  Eigen::Vector3d center;
  double sensing_range;
  double max_risk_level;
};

class PathVisualization : public rclcpp::Node {
public:
  PathVisualization();

private:
  struct DroneParams {
    int id;
    double start_x, start_y, start_z;
  };

  struct DroneData {
    Eigen::Vector3d start_pt;
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    path_manager::msg::PolyTraj current_traj;
  };

  void loadDroneParameters();
  void loadObstacleParameters();
  void logPositions();
  visualization_msgs::msg::Marker createMarker(const std::string& ns, int id, int type,
                                              double scale, float r, float g, float b, float a);
  void optimizedPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
  void globalPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
  void updatePosition();
  void publishPath(const std::vector<Eigen::Vector3d>& path, int id, float r, float g, float b, float alpha,
                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub);
  void publishObstacles();
  void simplePathCallback(const nav_msgs::msg::Path::SharedPtr msg, int drone_id);
  void publishTraveledPaths();
  void loadRiskZoneParameters();
  void publishRiskZones();

  int num_drones_;
  bool enable_obstacles_;
  std::vector<Obstacle> obstacle_centers_;
  std::vector<VisRiskZone> risk_zones_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr risk_zone_pub_;
  rclcpp::TimerBase::SharedPtr risk_zone_timer_;

  std::vector<DroneParams> drone_params_;
  std::vector<DroneData> drone_data_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr simple_path_marker_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> position_pubs_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> position_marker_pubs_;
  std::vector<rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr> optimized_path_subs_;
  std::vector<rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr> global_path_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> simple_path_subs_;
  std::vector<std::vector<Eigen::Vector3d>> traveled_paths_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> traveled_path_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
  rclcpp::TimerBase::SharedPtr traveled_path_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // PATH_VISUALIZATION_H