#include "path_visualization/path_visualization.h"
using namespace std::chrono_literals;

PathVisualization::PathVisualization() : Node("path_visualization")
{
  this->declare_parameter("obstacles", std::vector<double>{});
  std::vector<double> obstacle_params;
  this->get_parameter("obstacles", obstacle_params);
  if (obstacle_params.empty() || obstacle_params.size() % 3 != 0) {
    obstacle_centers_ = {
        Eigen::Vector3d(-2.0, -2.25, 0.5),
        Eigen::Vector3d(1.0, 0.0, 0.5),
        Eigen::Vector3d(0.0, 1.0, 0.5),
        Eigen::Vector3d(3.0, 3.0, 0.5)};
  } else {
    for (size_t i = 0; i < obstacle_params.size(); i += 3) {
      obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], 0.5);
    }
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", sensor_qos);
  optimized_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("opt_trajectory", sensor_qos);

  position_sub_ = this->create_subscription<px4_msgs::msg::Monitoring>(
      "/vehicle1/fmu/out/monitoring", sensor_qos,
      std::bind(&PathVisualization::positionCallback, this, std::placeholders::_1));

  optimized_path_sub_ = this->create_subscription<path_manager::msg::PolyTraj>(
      "/optimized_path", sensor_qos,
      std::bind(&PathVisualization::optimizedPathCallback, this, std::placeholders::_1));
}

visualization_msgs::msg::Marker PathVisualization::createMarker(const std::string &ns, int id, int type,
                                                                double scale, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  return marker;
}

void PathVisualization::positionCallback(const px4_msgs::msg::Monitoring::SharedPtr msg)
{
  start_pt_ = Eigen::Vector3d(msg->pos_x, msg->pos_y, 0.5);
}

void PathVisualization::optimizedPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg)
{
  std::vector<Eigen::Vector3d> optimized_path;
  double dt = 0.1;

  int piece_num = msg->duration.size();
  double total_time = 0.0;

  for (int i = 0; i < piece_num; ++i) {
    double duration = msg->duration[i];
    total_time += duration;
    int offset = i * (msg->order + 1);

    for (double t = 0.0; t <= duration; t += dt) {
      double x = 0.0, y = 0.0, z = 0.0;
      for (int j = 0; j <= msg->order; ++j) {
        double t_pow = std::pow(t, msg->order - j);
        x += msg->coef_x[offset + j] * t_pow;
        y += msg->coef_y[offset + j] * t_pow;
        z += msg->coef_z[offset + j] * t_pow;
      }
      optimized_path.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  publishPath(optimized_path, 0, 1.0, 0.0, 0.0, 1.0, optimized_traj_pub_);
  publishObstacles();
  RCLCPP_INFO(this->get_logger(), "Visualizing optimized path with %zu points, total duration: %.2f", optimized_path.size(), total_time);
}

void PathVisualization::publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b,
                                    float alpha, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  auto marker = createMarker("path", id, visualization_msgs::msg::Marker::LINE_STRIP, 0.05, r, g, b, alpha);
  for (const auto &pt : path) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    marker.points.push_back(p);
  }
  pub->publish(marker);
}

void PathVisualization::publishObstacles()
{
  for (size_t i = 0; i < obstacle_centers_.size(); ++i) {
    auto marker = createMarker("obstacle", i, visualization_msgs::msg::Marker::SPHERE, 1.0, 0.0, 1.0, 0.0, 0.5);
    marker.pose.position.x = obstacle_centers_[i].x();
    marker.pose.position.y = obstacle_centers_[i].y();
    marker.pose.position.z = obstacle_centers_[i].z();
    marker_pub_->publish(marker);
  }
}