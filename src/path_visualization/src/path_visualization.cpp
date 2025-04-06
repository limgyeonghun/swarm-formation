#include "path_visualization/path_visualization.h"
using namespace std::chrono_literals;

PathVisualization::PathVisualization() : Node("path_visualization"), current_time_(0.0)
{
  // Load obstacle parameters
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
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("current_position", sensor_qos);  // 현재 위치 퍼블리시
  position_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("position_markers", sensor_qos);  // 점 시각화

  position_sub_ = this->create_subscription<px4_msgs::msg::Monitoring>(
      "/vehicle1/fmu/out/monitoring", sensor_qos,
      std::bind(&PathVisualization::positionCallback, this, std::placeholders::_1));

  optimized_path_sub_ = this->create_subscription<path_manager::msg::PolyTraj>(
      "/optimized_path", sensor_qos,
      std::bind(&PathVisualization::optimizedPathCallback, this, std::placeholders::_1));

  // 0.01초 주기 타이머 추가
  timer_ = this->create_wall_timer(10ms, std::bind(&PathVisualization::updatePosition, this));

  publishObstacles();
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
  double total_duration = 0.0;
  for (int i = 0; i < piece_num; ++i) total_duration += msg->duration[i];

  for (int i = 0; i < piece_num; ++i) {
    double duration = msg->duration[i];
    int offset = i * (msg->order + 1);
    for (double t = 0.0; t <= duration; t += dt) {
      double x = 0.0, y = 0.0, z = 0.0;
      for (int j = 0; j <= msg->order; ++j) {
        double t_pow = std::pow(t, msg->order - j);  // 높은 차수부터
        x += msg->coef_x[offset + j] * t_pow;
        y += msg->coef_y[offset + j] * t_pow;
        z += msg->coef_z[offset + j] * t_pow;
      }
      optimized_path.push_back(Eigen::Vector3d(x, y, z));
    }
  }
  publishObstacles();
  publishPath(optimized_path, 0, 1.0, 0.0, 0.0, 1.0, optimized_traj_pub_);
  current_traj_ = *msg;  // 현재 경로 저장
  current_time_ = 0.0;   // 시간 초기화
  RCLCPP_INFO(this->get_logger(), "Received new trajectory with %zu pieces, duration: %.2f", piece_num, total_duration);
}

void PathVisualization::updatePosition()
{
  if (current_traj_.duration.empty()) return;

  double total_duration = 0.0;
  for (const auto& dur : current_traj_.duration) total_duration += dur;

  if (current_time_ <= total_duration) {
    double x = 0.0, y = 0.0, z = 0.0;
    double t_remaining = current_time_;
    for (int i = 0; i < current_traj_.duration.size(); ++i) {
      double duration = current_traj_.duration[i];
      if (t_remaining <= duration) {
        int offset = i * (current_traj_.order + 1);
        for (int j = 0; j <= current_traj_.order; ++j) {
          double t_pow = std::pow(t_remaining, current_traj_.order - j);
          x += current_traj_.coef_x[offset + j] * t_pow;
          y += current_traj_.coef_y[offset + j] * t_pow;
          z += current_traj_.coef_z[offset + j] * t_pow;
        }
        break;
      }
      t_remaining -= duration;
    }

    start_pt_ = Eigen::Vector3d(x, y, z);  // 현재 위치 업데이트
    // RCLCPP_INFO(this->get_logger(), "Position at t=%.2f: (%.2f, %.2f, %.2f)", current_time_, x, y, z);

    // 점 시각화
    auto marker = createMarker("position", 0, visualization_msgs::msg::Marker::POINTS, 0.1, 0.0, 0.0, 1.0, 1.0);
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    marker.points.push_back(p);
    position_marker_pub_->publish(marker);

    // 현재 위치 퍼블리시 (PathManager에서 사용 가능)
    geometry_msgs::msg::PointStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = this->now();
    pos_msg.point.x = x;
    pos_msg.point.y = y;
    pos_msg.point.z = z;
    position_pub_->publish(pos_msg);

    current_time_ += 0.01;  // 0.01초 증가
  }
}

void PathVisualization::publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b,
                                    float alpha, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  auto marker = createMarker("path", id, visualization_msgs::msg::Marker::LINE_STRIP, 0.05, r, g, b, alpha);
  for (const auto &pt : path) {
    geometry_msgs::msg::Point p;
    p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
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