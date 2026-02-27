#include "path_visualization/path_visualization.h"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

std::tuple<float, float, float> getDroneColor(int drone_id)
{
  switch (drone_id)
  {
  case 0:
    return {1.0f, 0.0f, 0.0f}; // red
  case 1:
    return {0.0f, 1.0f, 0.0f}; // green
  case 2:
    return {0.0f, 0.0f, 1.0f}; // blue
  case 3:
    return {1.0f, 1.0f, 0.0f}; // yellow
  case 4:
    return {1.0f, 0.0f, 1.0f}; // magenta
  case 5:
    return {0.0f, 1.0f, 1.0f}; // cyan
  default:
    return {0.5f, 0.5f, 0.5f}; // gray for any id >=6
  }
};

PathVisualization::PathVisualization() : Node("path_visualization")
{
  // Load obstacle avoidance parameter
  this->declare_parameter("enable_obstacles", true);
  this->get_parameter("enable_obstacles", enable_obstacles_);
  RCLCPP_INFO(this->get_logger(), "Obstacle visualization: %s", enable_obstacles_ ? "enabled" : "disabled");

  // Load drone parameters from drones.yaml
  loadDroneParameters();

  // Load obstacle parameters from obstacles.yaml
  loadObstacleParameters();

  // Load threat zone parameters
  loadThreatZones();

  // Load road boundary parameters from map.yaml
  loadRoadParameters();

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", sensor_qos);
  optimized_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("opt_trajectory", sensor_qos);
  global_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_trajectory", sensor_qos);
  simple_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("simple_path_trajectory", sensor_qos);
  road_boundary_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("road_boundary", sensor_qos);
  threat_field_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("threat_field", sensor_qos);

  position_pubs_.resize(num_drones_);
  position_marker_pubs_.resize(num_drones_);
  drone_data_.resize(num_drones_);
  simple_path_subs_.resize(num_drones_);
  optimized_path_subs_.resize(num_drones_);
  global_path_subs_.resize(num_drones_);

  for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
  {
    // Match FSM topic naming: /V1, /V2, etc. (drone_id+1)
    std::string topic_prefix = "/V" + std::to_string(drone_id + 1);

    std::string position_topic = topic_prefix + "/current_position";
    position_pubs_[drone_id] = this->create_publisher<geometry_msgs::msg::PointStamped>(position_topic, sensor_qos);
    position_marker_pubs_[drone_id] = this->create_publisher<visualization_msgs::msg::Marker>(
        "position_markers_drone_" + std::to_string(drone_id), sensor_qos);

    std::string simple_path_topic = topic_prefix + "/simple_path";
    simple_path_subs_[drone_id] = this->create_subscription<nav_msgs::msg::Path>(
        simple_path_topic, sensor_qos,
        [this, drone_id](const nav_msgs::msg::Path::SharedPtr msg)
        { this->simplePathCallback(msg, drone_id); });

    // Subscribe to per-drone optimized trajectory (match FSM naming)
    std::string optimized_traj_topic = topic_prefix + "/planning/trajectory";
    optimized_path_subs_[drone_id] = this->create_subscription<path_manager::msg::PolyTraj>(
        optimized_traj_topic, sensor_qos,
        std::bind(&PathVisualization::optimizedPathCallback, this, std::placeholders::_1));

    // Subscribe to per-drone global path (match FSM naming)
    std::string global_path_topic = topic_prefix + "/planning/global";
    global_path_subs_[drone_id] = this->create_subscription<path_manager::msg::PolyTraj>(
        global_path_topic, sensor_qos,
        std::bind(&PathVisualization::globalPathCallback, this, std::placeholders::_1));

    drone_data_[drone_id].start_pt = Eigen::Vector3d(
        drone_params_[drone_id].start_x,
        drone_params_[drone_id].start_y,
        drone_params_[drone_id].start_z);

    RCLCPP_INFO(this->get_logger(), "Drone %d: Subscribed to %s and %s",
                drone_id, optimized_traj_topic.c_str(), global_path_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drone %d: Initial position set to (%.2f, %.2f, %.2f)",
                drone_id, drone_data_[drone_id].start_pt.x(), drone_data_[drone_id].start_pt.y(), drone_data_[drone_id].start_pt.z());
  }

  timer_ = this->create_wall_timer(10ms, std::bind(&PathVisualization::updatePosition, this));
  log_timer_ = this->create_wall_timer(150ms, std::bind(&PathVisualization::logPositions, this));

  // Only publish obstacles if obstacle avoidance is enabled
  if (enable_obstacles_)
  {
    // Create a timer to periodically publish obstacles to ensure they remain visible
    obstacle_timer_ = this->create_wall_timer(1000ms, std::bind(&PathVisualization::publishObstacles, this));
    publishObstacles(); // Initial publish
  }
  
  // Publish road boundaries if enabled
  if (true)
  {
    publishRoadBoundaries();
  }

  // Publish threat field if enabled
  if (enable_threat_zones_)
  {
    threat_field_timer_ = this->create_wall_timer(2000ms, std::bind(&PathVisualization::publishThreatField, this));
    publishThreatField(); // Initial publish
  }
}

void PathVisualization::simplePathCallback(const nav_msgs::msg::Path::SharedPtr msg, int drone_id)
{
  if (drone_id < 0 || drone_id >= num_drones_)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid drone_id: %d", drone_id);
    return;
  }

  std::vector<Eigen::Vector3d> simple_path;
  for (const auto &pose : msg->poses)
  {
    simple_path.emplace_back(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
  }

  if (simple_path.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Drone %d: Empty simple path received", drone_id);
    return;
  }

  auto [r, g, b] = getDroneColor(drone_id);

  publishPath(simple_path, drone_id, r, g, b, 0.0, simple_path_marker_pub_);

  // RCLCPP_INFO(this->get_logger(), "Drone %d: Published simple path with %zu points", drone_id, simple_path.size());
}

void PathVisualization::loadDroneParameters()
{
  // Declare and get num_drones
  this->declare_parameter("num_drones", 1);

  if (!this->get_parameter("num_drones", num_drones_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load num_drones from drones.yaml");
    throw std::runtime_error("Failed to load num_drones");
  }

  drone_params_.resize(num_drones_);
  for (int i = 0; i < num_drones_; ++i)
  {
    std::string drone_key = "drone_" + std::to_string(i);
    DroneParams drone;

    // Declare and get drone parameters
    this->declare_parameter(drone_key + ".drone_id", i);
    this->declare_parameter(drone_key + ".start_point_x", 0.0);
    this->declare_parameter(drone_key + ".start_point_y", 0.0);
    this->declare_parameter(drone_key + ".start_point_z", 0.0);

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
  if (this->get_parameter("obstacles", obstacle_params))
  {
    if (obstacle_params.size() % 3 != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid obstacles.yaml format, size not divisible by 3");
      throw std::runtime_error("Invalid obstacles.yaml format");
    }
    for (size_t i = 0; i < obstacle_params.size(); i += 3)
    {
      obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], obstacle_params[i + 2]);
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load obstacles.yaml, using default obstacles");
    obstacle_centers_ = {
        Eigen::Vector3d(-2.0, -2.25, 0.5),
        Eigen::Vector3d(1.0, 0.0, 0.5),
        Eigen::Vector3d(0.0, 1.0, 0.5),
        Eigen::Vector3d(3.0, 3.0, 0.5)};
  }
  
  RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacles for visualization", obstacle_centers_.size());
}

void PathVisualization::logPositions()
{
  std::stringstream ss;
  ss << "Positions: ";
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
  {
    const auto &data = drone_data_[drone_id];
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
  if (drone_id < 0 || drone_id >= num_drones_)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid drone_id: %d", drone_id);
    return;
  }
  if (msg->coef_x.size() != msg->duration.size() * (msg->order + 1))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid trajectory coefficients for drone %d", drone_id);
    return;
  }
  
  auto &data = drone_data_[drone_id];
  
  std::vector<Eigen::Vector3d> optimized_path;
  double dt = 0.1;
  int piece_num = msg->duration.size();
  double total_duration = 0.0;
  for (int i = 0; i < piece_num; ++i)
    total_duration += msg->duration[i];
  if (total_duration < 0.1)
  {
    RCLCPP_WARN(this->get_logger(), "Drone %d: Trajectory too short (%.2f s)", drone_id, total_duration);
    return;
  }
  for (int i = 0; i < piece_num; ++i)
  {
    double duration = msg->duration[i];
    int offset = i * (msg->order + 1);
    for (double t = 0.0; t <= duration; t += dt)
    {
      double x = 0.0, y = 0.0, z = 0.0;
      for (int j = 0; j <= msg->order; ++j)
      {
        double t_pow = std::pow(t, msg->order - j);
        x += msg->coef_x[offset + j] * t_pow;
        y += msg->coef_y[offset + j] * t_pow;
        z += msg->coef_z[offset + j] * t_pow;
      }
      optimized_path.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  auto [r, g, b] = getDroneColor(drone_id);
  publishPath(optimized_path, drone_id, r, g, b, 1.0, optimized_traj_pub_);
  
  // Only publish obstacles if obstacle avoidance is enabled
  if (enable_obstacles_)
  {
    publishObstacles();
  }

  publishRoadBoundaries();

  Eigen::Vector3d current_pos = data.start_pt;
  double best_t = 0.0;
  double min_dist = std::numeric_limits<double>::max();
  
  // Find closest point on trajectory
  for (double t = 0.0; t <= total_duration; t += 0.01)
  {
    double x = 0.0, y = 0.0, z = 0.0;
    double t_remaining = t;
    for (int i = 0; i < piece_num; ++i)
    {
      double duration = msg->duration[i];
      if (t_remaining <= duration)
      {
        int offset = i * (msg->order + 1);
        for (int j = 0; j <= msg->order; ++j)
        {
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
    if (dist < min_dist)
    {
      min_dist = dist;
      best_t = t;
    }
  }
  // Handle large jumps by resetting to trajectory start
  if (min_dist > 2.0)
  {
    RCLCPP_WARN(this->get_logger(), "Drone %d: Large jump detected (%.2f m), resetting to trajectory start", drone_id, min_dist);
    best_t = 0.0;
    data.start_pt = Eigen::Vector3d(
        msg->coef_x[0], // Start point is just the first coefficient for position
        msg->coef_y[0],
        msg->coef_z[0]
    );
  }
  data.current_traj = *msg;
}

void PathVisualization::globalPathCallback(const path_manager::msg::PolyTraj::SharedPtr msg)
{
  std::vector<Eigen::Vector3d> global_path;
  double dt = 0.1;

  int piece_num = msg->duration.size();
  double total_duration = 0.0;
  for (int i = 0; i < piece_num; ++i)
    total_duration += msg->duration[i];

  for (int i = 0; i < piece_num; ++i)
  {
    double duration = msg->duration[i];
    int offset = i * (msg->order + 1);
    for (double t = 0.0; t <= duration; t += dt)
    {
      double x = 0.0, y = 0.0, z = 0.0;
      for (int j = 0; j <= msg->order; ++j)
      {
        double t_pow = std::pow(t, msg->order - j);
        x += msg->coef_x[offset + j] * t_pow;
        y += msg->coef_y[offset + j] * t_pow;
        z += msg->coef_z[offset + j] * t_pow;
      }
      global_path.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  auto [r, g, b] = getDroneColor(msg->drone_id);
  publishPath(global_path, msg->drone_id, r, g, b, 0.8, global_traj_pub_);
}

void PathVisualization::updatePosition()
{
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
  {
    auto &data = drone_data_[drone_id];
    double x, y, z;
    if (data.current_traj.duration.empty())
    {
      x = data.start_pt.x();
      y = data.start_pt.y();
      z = data.start_pt.z();
    }
    else
    {
      double current_ros_time = this->now().seconds();
      double trajectory_start_time = data.current_traj.start_time.sec + data.current_traj.start_time.nanosec * 1e-9;
      double t_cur = current_ros_time - trajectory_start_time;
      
      double total_duration = 0.0;
      for (const auto &dur : data.current_traj.duration)
        total_duration += dur;

      t_cur = std::max(0.0, std::min(t_cur, total_duration));
      
      if (t_cur <= total_duration)
      {
        double t_remaining = t_cur;
        for (size_t i = 0; i < data.current_traj.duration.size(); ++i)
        {
          double duration = data.current_traj.duration[i];
          if (t_remaining <= duration)
          {
            int offset = i * (data.current_traj.order + 1);
            x = 0.0, y = 0.0, z = 0.0;
            for (int j = 0; j <= data.current_traj.order; ++j)
            {
              double t_pow = std::pow(t_remaining, data.current_traj.order - j);
              x += data.current_traj.coef_x[offset + j] * t_pow;
              y += data.current_traj.coef_y[offset + j] * t_pow;
              z += data.current_traj.coef_z[offset + j] * t_pow;
            }
            break;
          }
          t_remaining -= duration;
        }
      }
      else
      {
        int last_piece = data.current_traj.duration.size() - 1;
        double last_duration = data.current_traj.duration[last_piece];
        int offset = last_piece * (data.current_traj.order + 1);
        x = 0.0, y = 0.0, z = 0.0;
        for (int j = 0; j <= data.current_traj.order; ++j)
        {
          double t_pow = std::pow(last_duration, data.current_traj.order - j);
          x += data.current_traj.coef_x[offset + j] * t_pow;
          y += data.current_traj.coef_y[offset + j] * t_pow;
          z += data.current_traj.coef_z[offset + j] * t_pow;
        }
      }
      data.start_pt = Eigen::Vector3d(x, y, z);
    }
    auto [r, g, b] = getDroneColor(drone_id);
    auto marker = createMarker("position_drone_" + std::to_string(drone_id), 0,
                               visualization_msgs::msg::Marker::POINTS, 0.3, r, g, b, 1.0);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
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
  for (const auto &pt : path)
  {
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
  // Only publish obstacles if obstacle avoidance is enabled
  if (!enable_obstacles_)
  {
    return;
  }
  
  // Create a single marker array for all obstacles to improve performance
  auto marker = createMarker("obstacles", 0, visualization_msgs::msg::Marker::CUBE_LIST, 1.5, 0.0, 1.0, 0.0, 0.7);
  
  // Set lifetime to ensure markers don't disappear
  marker.lifetime = rclcpp::Duration::from_seconds(2.0);
  
  // Add all obstacle positions to the marker
  for (size_t i = 0; i < obstacle_centers_.size(); ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = obstacle_centers_[i].x();
    point.y = obstacle_centers_[i].y();
    point.z = obstacle_centers_[i].z();
    marker.points.push_back(point);
  }
  
  // Publish the single marker containing all obstacles
  marker_pub_->publish(marker);
  
  RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacles", obstacle_centers_.size());
}

void PathVisualization::loadRoadParameters()
{
  // Load road boundary parameters from map.yaml
  this->declare_parameter("grid_map/use_road_boundary", false);
  this->declare_parameter("grid_map/road_width", 8.0);
  this->declare_parameter("grid_map/road_center_x", 0.0);
  this->declare_parameter("grid_map/road_margin", 0.5);
  this->declare_parameter("grid_map/map_size_y", 120.0);
  this->declare_parameter("grid_map/road_segments", std::vector<double>{});
  
  this->get_parameter("grid_map/use_road_boundary", use_road_boundary_);
  this->get_parameter("grid_map/road_width", road_width_);
  this->get_parameter("grid_map/road_center_x", road_center_x_);
  this->get_parameter("grid_map/road_margin", road_margin_);
  this->get_parameter("grid_map/map_size_y", map_size_y_);

  std::vector<double> road_segments_flat;
  if (this->get_parameter("grid_map/road_segments", road_segments_flat)) {
    if (road_segments_flat.size() % 5 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid road_segments format: size must be multiple of 5");
      return;
    }

    for (size_t i = 0; i < road_segments_flat.size(); i += 5) {
      std::vector<double> segment;
      segment.push_back(road_segments_flat[i]);     // start_x
      segment.push_back(road_segments_flat[i + 1]); // start_y
      segment.push_back(road_segments_flat[i + 2]); // end_x
      segment.push_back(road_segments_flat[i + 3]); // end_y
      segment.push_back(road_segments_flat[i + 4]); // width
      road_segments_.push_back(segment);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Road boundary visualization: %s", use_road_boundary_ ? "enabled" : "disabled");
  if (use_road_boundary_) {
    RCLCPP_INFO(this->get_logger(), "Road center_x: %.1f m", road_center_x_);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu road segments:", road_segments_.size());
    for (size_t i = 0; i < road_segments_.size(); ++i) {
      const auto& seg = road_segments_[i];
      RCLCPP_INFO(this->get_logger(), "  Segment %zu: (%.1f, %.1f) -> (%.1f, %.1f), width: %.1f m",
                  i, seg[0], seg[1], seg[2], seg[3], seg[4]);
    }
  }
}

void PathVisualization::publishRoadBoundaries()
{
  if (!use_road_boundary_) {
    return;
  }

  if (!road_segments_.empty()) {
    int marker_id = 0;

    for (const auto& segment : road_segments_) {
      double start_x = segment[0];
      double start_y = segment[1];
      double end_x = segment[2];
      double end_y = segment[3];

      double dx = end_x - start_x;
      double dy = end_y - start_y;
      double length = std::sqrt(dx*dx + dy*dy);
      if (length < 1e-6) continue;

      double nx = -dy / length;
      double ny = dx / length;
      
      double half_width = segment[4] / 2.0;

      auto left_marker = createMarker("road_boundary", marker_id++, 
        visualization_msgs::msg::Marker::LINE_STRIP, 0.3, 0.0, 0.0, 0.0, 1.0);
      geometry_msgs::msg::Point p1, p2;

      p1.x = start_x + nx * half_width - dx * 0.5; 
      p1.y = start_y + ny * half_width - dy * 0.5;
      p1.z = 0.5;
      left_marker.points.push_back(p1);

      p1.x = start_x + nx * half_width; 
      p1.y = start_y + ny * half_width; 
      p1.z = 0.5;
      p2.x = end_x + nx * half_width; 
      p2.y = end_y + ny * half_width; 
      p2.z = 0.5;
      left_marker.points.push_back(p1);
      left_marker.points.push_back(p2);

      p2.x = end_x + nx * half_width + dx * 0.5;
      p2.y = end_y + ny * half_width + dy * 0.5;
      p2.z = 0.5;
      left_marker.points.push_back(p2);
      
      road_boundary_pub_->publish(left_marker);

      auto right_marker = createMarker("road_boundary", marker_id++,
        visualization_msgs::msg::Marker::LINE_STRIP, 0.3, 0.0, 0.0, 0.0, 1.0);

      p1.x = start_x - nx * half_width - dx * 0.5;
      p1.y = start_y - ny * half_width - dy * 0.5;
      p1.z = 0.5;
      right_marker.points.push_back(p1);

      p1.x = start_x - nx * half_width;
      p1.y = start_y - ny * half_width;
      p1.z = 0.5;
      p2.x = end_x - nx * half_width;
      p2.y = end_y - ny * half_width;
      p2.z = 0.5;
      right_marker.points.push_back(p1);
      right_marker.points.push_back(p2);

      p2.x = end_x - nx * half_width + dx * 0.5;
      p2.y = end_y - ny * half_width + dy * 0.5;
      p2.z = 0.5;
      right_marker.points.push_back(p2);
      
      road_boundary_pub_->publish(right_marker);
    }
  }

  else {
    double road_half_width = road_width_ / 2.0;
    double road_left = road_center_x_ - road_half_width;
    double road_right = road_center_x_ + road_half_width;

    auto left_marker = createMarker("road_boundary", 0, 
      visualization_msgs::msg::Marker::LINE_STRIP, 0.3, 0.0, 0.0, 0.0, 1.0);
    geometry_msgs::msg::Point p1, p2;
    p1.x = road_left; p1.y = -map_size_y_/2.0; p1.z = 0.5;
    p2.x = road_left; p2.y = map_size_y_/2.0; p2.z = 0.5;
    left_marker.points.push_back(p1);
    left_marker.points.push_back(p2);
    road_boundary_pub_->publish(left_marker);

    auto right_marker = createMarker("road_boundary", 1,
      visualization_msgs::msg::Marker::LINE_STRIP, 0.3, 0.0, 0.0, 0.0, 1.0);
    p1.x = road_right; p1.y = -map_size_y_/2.0; p1.z = 0.5;
    p2.x = road_right; p2.y = map_size_y_/2.0; p2.z = 0.5;
    right_marker.points.push_back(p1);
    right_marker.points.push_back(p2);
    road_boundary_pub_->publish(right_marker);

  }
}

void PathVisualization::loadThreatZones()
{
  this->declare_parameter("enable_threat_zones", false);
  this->declare_parameter("threat_visualization_resolution", 2.0);
  this->declare_parameter("threat_zones", std::vector<double>());

  this->get_parameter("enable_threat_zones", enable_threat_zones_);
  this->get_parameter("threat_visualization_resolution", threat_visualization_resolution_);

  std::vector<double> threat_zones_flat;
  if (this->get_parameter("threat_zones", threat_zones_flat)) {
    // Format: [center_x, center_y, center_z, detection_range, engagement_range, max_threat_level, ...]
    if (threat_zones_flat.size() % 6 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid threat_zones format: size must be multiple of 6");
      return;
    }

    for (size_t i = 0; i < threat_zones_flat.size(); i += 6) {
      ThreatZoneViz zone;
      zone.center = Eigen::Vector3d(threat_zones_flat[i], threat_zones_flat[i + 1], threat_zones_flat[i + 2]);
      zone.detection_range = threat_zones_flat[i + 3];
      zone.engagement_range = threat_zones_flat[i + 4];
      zone.max_threat_level = threat_zones_flat[i + 5];
      zone.name = "ThreatZone_" + std::to_string(i / 6);
      threat_zones_.push_back(zone);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Threat zones visualization: %s", enable_threat_zones_ ? "enabled" : "disabled");
  if (enable_threat_zones_) {
    RCLCPP_INFO(this->get_logger(), "Loaded %zu threat zones (resolution: %.1fm):",
                threat_zones_.size(), threat_visualization_resolution_);
    for (size_t i = 0; i < threat_zones_.size(); ++i) {
      const auto& zone = threat_zones_[i];
      RCLCPP_INFO(this->get_logger(), "  Zone %zu: center=(%.1f, %.1f, %.1f), detection=%.1fm, engagement=%.1fm, threat=%.1f",
                  i, zone.center.x(), zone.center.y(), zone.center.z(),
                  zone.detection_range, zone.engagement_range, zone.max_threat_level);
    }
  }
}

void PathVisualization::publishThreatField()
{
  if (!enable_threat_zones_ || threat_zones_.empty()) {
    return;
  }

  // Helper function: Calculate threat level at a given position (Gaussian decay)
  auto calculateThreat = [this](const Eigen::Vector3d& pos) -> double {
    double total_threat = 0.0;
    for (const auto& zone : threat_zones_) {
      Eigen::Vector3d diff = pos - zone.center;
      double dist = diff.norm();

      // Gaussian decay within engagement range
      if (dist < zone.engagement_range) {
        double sigma = zone.engagement_range / 3.0;
        double normalized_dist = dist / sigma;
        total_threat += zone.max_threat_level * exp(-0.5 * normalized_dist * normalized_dist);
      }
      // Linear decay in detection range
      else if (dist < zone.detection_range) {
        double ratio = (dist - zone.engagement_range) /
                      (zone.detection_range - zone.engagement_range);
        total_threat += zone.max_threat_level * 0.3 * (1.0 - ratio);
      }
    }
    return total_threat;
  };

  // Helper function: Convert threat level to RGB color (gradient visualization)
  auto threatToColor = [](double threat, double max_threat) -> std::tuple<float, float, float> {
    // Normalize threat to [0, 1]
    double normalized = std::min(threat / max_threat, 1.0);

    // Color gradient: Blue (safe) -> Green -> Yellow -> Red (dangerous)
    float r, g, b;
    if (normalized < 0.25) {
      // Blue to Cyan
      float t = normalized / 0.25;
      r = 0.0;
      g = t * 0.5;
      b = 1.0;
    } else if (normalized < 0.5) {
      // Cyan to Green
      float t = (normalized - 0.25) / 0.25;
      r = 0.0;
      g = 0.5 + t * 0.5;
      b = 1.0 - t;
    } else if (normalized < 0.75) {
      // Green to Yellow
      float t = (normalized - 0.5) / 0.25;
      r = t;
      g = 1.0;
      b = 0.0;
    } else {
      // Yellow to Red
      float t = (normalized - 0.75) / 0.25;
      r = 1.0;
      g = 1.0 - t;
      b = 0.0;
    }
    return {r, g, b};
  };

  // Find max threat for normalization
  double max_threat_value = 0.0;
  for (const auto& zone : threat_zones_) {
    max_threat_value = std::max(max_threat_value, zone.max_threat_level);
  }

  // Visualize threat zones using triangle mesh with gradient coloring
  // Create sphere surface as triangulated mesh for smooth gradient
  auto marker = createMarker("threat_field", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST,
                            1.0, 1.0, 0.0, 0.0, 0.5);
  marker.lifetime = rclcpp::Duration::from_seconds(3.0);

  for (const auto& zone : threat_zones_) {
    // Create multiple concentric sphere shells at different radii
    int num_radial_layers = 5;  // Number of concentric shells
    int num_latitude = 16;      // Latitude divisions
    int num_longitude = 32;     // Longitude divisions

    for (int layer = 0; layer < num_radial_layers; ++layer) {
      double radius_ratio = (layer + 1.0) / num_radial_layers;
      double radius = zone.detection_range * radius_ratio;

      // Generate sphere mesh using latitude/longitude grid
      for (int lat = 0; lat < num_latitude; ++lat) {
        for (int lon = 0; lon < num_longitude; ++lon) {
          // Calculate 4 corners of current quad
          double phi1 = M_PI * lat / num_latitude;
          double phi2 = M_PI * (lat + 1) / num_latitude;
          double theta1 = 2.0 * M_PI * lon / num_longitude;
          double theta2 = 2.0 * M_PI * (lon + 1) / num_longitude;

          // Four vertices of the quad
          Eigen::Vector3d v1(radius * sin(phi1) * cos(theta1),
                             radius * sin(phi1) * sin(theta1),
                             radius * cos(phi1));
          Eigen::Vector3d v2(radius * sin(phi1) * cos(theta2),
                             radius * sin(phi1) * sin(theta2),
                             radius * cos(phi1));
          Eigen::Vector3d v3(radius * sin(phi2) * cos(theta2),
                             radius * sin(phi2) * sin(theta2),
                             radius * cos(phi2));
          Eigen::Vector3d v4(radius * sin(phi2) * cos(theta1),
                             radius * sin(phi2) * sin(theta1),
                             radius * cos(phi2));

          // Calculate threat and color for each vertex
          auto getVertexColor = [&](const Eigen::Vector3d& local_pos) {
            Eigen::Vector3d world_pos = zone.center + local_pos;
            double threat = calculateThreat(world_pos);

            auto [r, g, b] = threatToColor(threat, max_threat_value);
            std_msgs::msg::ColorRGBA color;
            color.r = r;
            color.g = g;
            color.b = b;
            double normalized_threat = std::min(threat / max_threat_value, 1.0);
            color.a = 0.01 + normalized_threat * 0.35;  // 0.15~0.5
            return color;
          };

          // Skip if all vertices have very low threat
          double avg_threat = (calculateThreat(zone.center + v1) +
                              calculateThreat(zone.center + v2) +
                              calculateThreat(zone.center + v3) +
                              calculateThreat(zone.center + v4)) / 4.0;
          if (avg_threat < 1.0) continue;

          // Create two triangles for this quad
          geometry_msgs::msg::Point p1, p2, p3, p4;
          p1.x = zone.center.x() + v1.x(); p1.y = zone.center.y() + v1.y(); p1.z = zone.center.z() + v1.z();
          p2.x = zone.center.x() + v2.x(); p2.y = zone.center.y() + v2.y(); p2.z = zone.center.z() + v2.z();
          p3.x = zone.center.x() + v3.x(); p3.y = zone.center.y() + v3.y(); p3.z = zone.center.z() + v3.z();
          p4.x = zone.center.x() + v4.x(); p4.y = zone.center.y() + v4.y(); p4.z = zone.center.z() + v4.z();

          // Triangle 1: v1, v2, v3
          marker.points.push_back(p1);
          marker.points.push_back(p2);
          marker.points.push_back(p3);
          marker.colors.push_back(getVertexColor(v1));
          marker.colors.push_back(getVertexColor(v2));
          marker.colors.push_back(getVertexColor(v3));

          // Triangle 2: v1, v3, v4
          marker.points.push_back(p1);
          marker.points.push_back(p3);
          marker.points.push_back(p4);
          marker.colors.push_back(getVertexColor(v1));
          marker.colors.push_back(getVertexColor(v3));
          marker.colors.push_back(getVertexColor(v4));
        }
      }
    }
  }

  threat_field_pub_->publish(marker);

  RCLCPP_DEBUG(this->get_logger(), "Published threat field with %zu triangles", marker.points.size() / 3);
}