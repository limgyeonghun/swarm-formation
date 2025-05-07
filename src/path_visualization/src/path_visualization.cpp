#include "path_visualization/path_visualization.h"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

PathVisualization::PathVisualization() : Node("path_visualization")
{
  // Load drone parameters from drones.yaml
  loadDroneParameters();

  // Load obstacle parameters from obstacles.yaml
  loadObstacleParameters();

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", sensor_qos);
  optimized_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("opt_trajectory", sensor_qos);
  global_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_trajectory", sensor_qos);

  position_pubs_.resize(num_drones_);
  position_marker_pubs_.resize(num_drones_);
  drone_data_.resize(num_drones_);
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id) {
    std::string position_topic = "/drone_" + std::to_string(drone_id) + "/current_position";
    position_pubs_[drone_id] = this->create_publisher<geometry_msgs::msg::PointStamped>(position_topic, sensor_qos);
    position_marker_pubs_[drone_id] = this->create_publisher<visualization_msgs::msg::Marker>(
        "position_markers_drone_" + std::to_string(drone_id), sensor_qos);

    drone_data_[drone_id].start_pt = Eigen::Vector3d(
        drone_params_[drone_id].start_x,
        drone_params_[drone_id].start_y,
        drone_params_[drone_id].start_z);
    drone_data_[drone_id].current_time = 0.0;

    RCLCPP_INFO(this->get_logger(), "Drone %d: Initial position set to (%.2f, %.2f, %.2f)",
                drone_id, drone_data_[drone_id].start_pt.x(), drone_data_[drone_id].start_pt.y(), drone_data_[drone_id].start_pt.z());
  }

  optimized_path_sub_ = this->create_subscription<path_manager::msg::PolyTraj>(
      "/planning/trajectory", sensor_qos,
      std::bind(&PathVisualization::optimizedPathCallback, this, std::placeholders::_1));

  global_path_sub_ = this->create_subscription<path_manager::msg::PolyTraj>(
      "/planning/global", sensor_qos,
      std::bind(&PathVisualization::globalPathCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(10ms, std::bind(&PathVisualization::updatePosition, this));
  log_timer_ = this->create_wall_timer(150ms, std::bind(&PathVisualization::logPositions, this));

  publishObstacles();
}

void PathVisualization::loadDroneParameters()
{
  // Declare and get num_drones
  this->declare_parameter("num_drones", 1);
  
  if (!this->get_parameter("num_drones", num_drones_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load num_drones from drones.yaml");
    throw std::runtime_error("Failed to load num_drones");
  }

  drone_params_.resize(num_drones_);
  for (int i = 0; i < num_drones_; ++i) {
    std::string drone_key = "drone_" + std::to_string(i);
    DroneParams drone;

    // Declare and get drone parameters
    this->declare_parameter(drone_key + ".drone_id", i);
    this->declare_parameter(drone_key + ".start_point_x", 0.0);
    this->declare_parameter(drone_key + ".start_point_y", 0.0);
    this->declare_parameter(drone_key + ".start_point_z", 0.5);

    if (!this->get_parameter(drone_key + ".drone_id", drone.id) ||
        !this->get_parameter(drone_key + ".start_point_x", drone.start_x) ||
        !this->get_parameter(drone_key + ".start_point_y", drone.start_y) ||
        !this->get_parameter(drone_key + ".start_point_z", drone.start_z))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load parameters for drone_%d", i);
      throw std::runtime_error("Failed to load drone parameters");
    }

    drone_params_[i] = drone;
    RCLCPP_INFO(this->get_logger(), "Drone %d: id=%d, start_point=(%.2f, %.2f, %.2f)",
                i, drone.id, drone.start_x, drone.start_y, drone.start_z);
  }
}

void PathVisualization::loadObstacleParameters()
{
  // Declare and get obstacles as a vector of doubles
  this->declare_parameter("obstacles", std::vector<double>{});
  std::vector<double> obstacle_params;
  if (this->get_parameter("obstacles", obstacle_params)) {
    if (obstacle_params.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid obstacles.yaml format, size not divisible by 3");
      throw std::runtime_error("Invalid obstacles.yaml format");
    }
    for (size_t i = 0; i < obstacle_params.size(); i += 3) {
      obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], obstacle_params[i + 2]);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load obstacles.yaml, using default obstacles");
    obstacle_centers_ = {
        Eigen::Vector3d(-2.0, -2.25, 0.5),
        Eigen::Vector3d(1.0, 0.0, 0.5),
        Eigen::Vector3d(0.0, 1.0, 0.5),
        Eigen::Vector3d(3.0, 3.0, 0.5)
    };
  }
}

void PathVisualization::logPositions()
{
  std::stringstream ss;
  ss << "Positions: ";
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id) {
    const auto& data = drone_data_[drone_id];
    ss << "Drone " << drone_id << ": (" << std::fixed << std::setprecision(2)
       << data.start_pt(0) << ", " << data.start_pt(1) << ", " << data.start_pt(2) << ") ";
  }
  // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
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

void PathVisualization::optimizedPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg)
{
  int drone_id = msg->drone_id;
  if (drone_id < 0 || drone_id >= num_drones_) {
    RCLCPP_WARN(this->get_logger(), "Invalid drone_id: %d", drone_id);
    return;
  }
  if (msg->coef_x.size() != msg->duration.size() * (msg->order + 1)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid trajectory coefficients for drone %d", drone_id);
    return;
  }
  auto& data = drone_data_[drone_id];
  std::vector<Eigen::Vector3d> optimized_path;
  double dt = 0.1;
  int piece_num = msg->duration.size();
  double total_duration = 0.0;
  for (int i = 0; i < piece_num; ++i) total_duration += msg->duration[i];
  if (total_duration < 0.1) {
    RCLCPP_WARN(this->get_logger(), "Drone %d: Trajectory too short (%.2f s)", drone_id, total_duration);
    return;
  }
  for (int i = 0; i < piece_num; ++i) {
    double duration = msg->duration[i];
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
  float r = (drone_id == 0) ? 1.0 : (drone_id == 1) ? 0.0 : 0.0;
  float g = (drone_id == 0) ? 0.0 : (drone_id == 1) ? 1.0 : 0.0;
  float b = (drone_id == 0) ? 0.0 : (drone_id == 1) ? 0.0 : 1.0;
  if(drone_id == 3) {
    r = 0.5; g = 0.5; b = 0.5;
  }
  publishPath(optimized_path, drone_id, r, g, b, 0.5, optimized_traj_pub_);
  publishObstacles();

  Eigen::Vector3d current_pos = data.start_pt;
  double best_t = 0.0;
  double min_dist = std::numeric_limits<double>::max();
  for (double t = 0.0; t <= total_duration; t += 0.01) {
    double x = 0.0, y = 0.0, z = 0.0;
    double t_remaining = t;
    for (int i = 0; i < piece_num; ++i) {
      double duration = msg->duration[i];
      if (t_remaining <= duration) {
        int offset = i * (msg->order + 1);
        for (int j = 0; j <= msg->order; ++j) {
          double t_pow = std::pow(t_remaining, msg->order - j);
          x += msg->coef_x[offset + j] * t_pow;
          y += msg->coef_y[offset + j] * t_pow;
          z += msg->coef_z[offset + j] * t_pow;
        }
        break;
      }
      t_remaining -= duration;
    }
    Eigen::Vector3d traj_pos(x, y, z);
    double dist = (traj_pos - current_pos).norm();
    if (dist < min_dist) {
      min_dist = dist;
      best_t = t;
    }
  }
  if (min_dist > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Drone %d: Large jump (%.2f m), keeping previous trajectory", drone_id, min_dist);
    return;
  }
  data.current_traj = *msg;
  data.current_time = best_t;
  // RCLCPP_INFO(this->get_logger(), "Drone %d: New trajectory at t=%.2f", drone_id, best_t);
}

void PathVisualization::globalPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg)
{
  std::vector<Eigen::Vector3d> global_path;
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
        double t_pow = std::pow(t, msg->order - j);
        x += msg->coef_x[offset + j] * t_pow;
        y += msg->coef_y[offset + j] * t_pow;
        z += msg->coef_z[offset + j] * t_pow;
      }
      global_path.push_back(Eigen::Vector3d(x, y, z));
    }
  }
}

void PathVisualization::updatePosition()
{
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id) {
    auto& data = drone_data_[drone_id];
    double x, y, z;
    if (data.current_traj.duration.empty()) {
      x = data.start_pt.x();
      y = data.start_pt.y();
      z = data.start_pt.z();
    } else {
      double total_duration = 0.0;
      for (const auto& dur : data.current_traj.duration) total_duration += dur;
      if (data.current_time <= total_duration) {
        double t_remaining = data.current_time;
        for (size_t i = 0; i < data.current_traj.duration.size(); ++i) {
          double duration = data.current_traj.duration[i];
          if (t_remaining <= duration) {
            int offset = i * (data.current_traj.order + 1);
            x = 0.0, y = 0.0, z = 0.0;
            for (int j = 0; j <= data.current_traj.order; ++j) {
              double t_pow = std::pow(t_remaining, data.current_traj.order - j);
              x += data.current_traj.coef_x[offset + j] * t_pow;
              y += data.current_traj.coef_y[offset + j] * t_pow;
              z += data.current_traj.coef_z[offset + j] * t_pow;
            }
            break;
          }
          t_remaining -= duration;
        }
      } else {
        int last_piece = data.current_traj.duration.size() - 1;
        double last_duration = data.current_traj.duration[last_piece];
        int offset = last_piece * (data.current_traj.order + 1);
        x = 0.0, y = 0.0, z = 0.0;
        for (int j = 0; j <= data.current_traj.order; ++j) {
          double t_pow = std::pow(last_duration, data.current_traj.order - j);
          x += data.current_traj.coef_x[offset + j] * t_pow;
          y += data.current_traj.coef_y[offset + j] * t_pow;
          z += data.current_traj.coef_z[offset + j] * t_pow;
        }
      }
      data.start_pt = Eigen::Vector3d(x, y, z);
      data.current_time += 0.01;
    }
    float r = (drone_id == 0) ? 1.0 : (drone_id == 1) ? 0.0 : 0.0;
    float g = (drone_id == 0) ? 0.0 : (drone_id == 1) ? 1.0 : 0.0;
    float b = (drone_id == 0) ? 0.0 : (drone_id == 1) ? 0.0 : 1.0;
    if(drone_id == 3) {
      r = 0.5; g = 0.5; b = 0.5;
    }
    auto marker = createMarker("position_drone_" + std::to_string(drone_id), 0,
                              visualization_msgs::msg::Marker::POINTS, 0.3, r, g, b, 1.0);
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    marker.points.push_back(p);
    position_marker_pubs_[drone_id]->publish(marker);
    geometry_msgs::msg::PointStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = this->now();
    pos_msg.point.x = x;
    pos_msg.point.y = y;
    pos_msg.point.z = z;
    position_pubs_[drone_id]->publish(pos_msg);
  }
}

void PathVisualization::publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                                   const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  std::string ns_prefix = (pub == global_traj_pub_) ? "global_path_drone_" : "opt_path_drone_";
  auto marker = createMarker(ns_prefix + std::to_string(id), id,
                            visualization_msgs::msg::Marker::LINE_STRIP, 0.05, r, g, b, alpha);
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