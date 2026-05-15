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
    return {0.0f, 1.0f, 0.0f}; // green
  case 1:
    return {0.0f, 0.0f, 1.0f}; // blue
  case 2:
    return {1.0f, 1.0f, 0.0f}; // yellow
  case 3:
    return {1.0f, 0.0f, 1.0f}; // magenta
  case 4:
    return {0.0f, 1.0f, 1.0f}; // cyan
  case 5:
    return {1.0f, 0.5f, 0.0f}; // orange
  default:
    return {0.5f, 0.5f, 0.5f}; // gray for any id >=6
  }
};

PathVisualization::PathVisualization() : Node("path_visualization")
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Load obstacle avoidance parameter
  this->declare_parameter("enable_obstacles", true);
  this->get_parameter("enable_obstacles", enable_obstacles_);
  RCLCPP_INFO(this->get_logger(), "Obstacle visualization: %s", enable_obstacles_ ? "enabled" : "disabled");

  // Load drone parameters from drones.yaml
  loadDroneParameters();

  // Load obstacle parameters from obstacles.yaml
  loadObstacleParameters();

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", sensor_qos);
  optimized_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("opt_trajectory", sensor_qos);
  global_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_trajectory", sensor_qos);
  simple_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("simple_path_trajectory", sensor_qos);
  position_pubs_.resize(num_drones_);
  position_marker_pubs_.resize(num_drones_);
  drone_data_.resize(num_drones_);
  simple_path_subs_.resize(num_drones_);
  optimized_path_subs_.resize(num_drones_);
  global_path_subs_.resize(num_drones_);
  traveled_paths_.resize(num_drones_);
  traveled_path_pubs_.resize(num_drones_);

  for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
  {
    // Match FSM topic naming: /V1, /V2, etc. (drone_id+1)
    std::string topic_prefix = "/V" + std::to_string(drone_id + 1);

    std::string position_topic = topic_prefix + "/current_position";
    position_pubs_[drone_id] = this->create_publisher<geometry_msgs::msg::PointStamped>(position_topic, sensor_qos);
    position_marker_pubs_[drone_id] = this->create_publisher<visualization_msgs::msg::Marker>(
        "position_markers_drone_" + std::to_string(drone_id), sensor_qos);
    traveled_path_pubs_[drone_id] = this->create_publisher<visualization_msgs::msg::Marker>(
        "traveled_path_drone_" + std::to_string(drone_id), sensor_qos);

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
  traveled_path_timer_ = this->create_wall_timer(200ms, std::bind(&PathVisualization::publishTraveledPaths, this));

  // Only publish obstacles if obstacle avoidance is enabled
  if (enable_obstacles_)
  {
    // Create a timer to periodically publish obstacles to ensure they remain visible
    obstacle_timer_ = this->create_wall_timer(1000ms, std::bind(&PathVisualization::publishObstacles, this));
    publishObstacles(); // Initial publish
  }

  // Load static risk zones from launch params (legacy path) and create
  // the publisher / timer unconditionally so runtime updates via the
  // /risk_zones/load topic can become visible without restart.
  loadRiskZoneParameters();
  // QoS::TransientLocal so a late RViz subscriber still receives the
  // most recent zone marker set without us having to republish.
  rclcpp::QoS marker_qos(10);
  marker_qos.transient_local().reliable();
  risk_zone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "risk_field", marker_qos);
  // No periodic timer: markers carry lifetime=0 (never expire in RViz)
  // and we only need to republish when the zone set changes via the
  // /risk_zones/load callback below.
  risk_zone_sub_ = this->create_subscription<path_manager::msg::RiskZoneArray>(
      "/risk_zones/load", rclcpp::QoS(1).reliable(),
      std::bind(&PathVisualization::riskZoneArrayCallback, this, std::placeholders::_1));
  publishRiskZones();
  RCLCPP_INFO(this->get_logger(),
              "Risk zone visualization: %zu zones from launch params; "
              "subscribed to /risk_zones/load for runtime updates",
              risk_zones_.size());
}

void PathVisualization::riskZoneArrayCallback(
    const path_manager::msg::RiskZoneArray::SharedPtr msg)
{
  risk_zones_.clear();
  risk_zones_.reserve(msg->zones.size());
  for (const auto &z : msg->zones) {
    if (z.reach <= 0.0 || z.peak <= 0.0) continue;
    VisRiskZone tz;
    tz.center = Eigen::Vector3d(z.center.x, z.center.y, z.center.z);
    tz.reach = z.reach;
    tz.peak = std::min(z.peak, 1.0);
    risk_zones_.push_back(tz);
  }
  // The zone set just changed — wipe the previous markers once, then
  // let publishRiskZones() draw the new (or empty) set.
  if (risk_zone_pub_) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = this->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    risk_zone_pub_->publish(del);
  }
  RCLCPP_INFO(this->get_logger(),
              "[risk_zones] runtime update: %zu zones visualized",
              risk_zones_.size());
  publishRiskZones();
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

  // Use dark blue color for simple path (local path)
  publishPath(simple_path, drone_id, 0.12f, 0.39f, 1.0f, 0.7, simple_path_marker_pub_);

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
  this->declare_parameter("obstacles", std::vector<double>{});
  std::vector<double> obstacle_params;
  if (this->get_parameter("obstacles", obstacle_params))
  {
    size_t i = 0;
    while (i < obstacle_params.size())
    {
      if (i + 2 >= obstacle_params.size()) break;

      Eigen::Vector3d center(obstacle_params[i], obstacle_params[i + 1], obstacle_params[i + 2]);

      if (i + 3 < obstacle_params.size())
      {
        int shape_type = static_cast<int>(obstacle_params[i + 3]);

        if (shape_type == 0 && i + 4 < obstacle_params.size())  // CIRCLE
        {
          double radius = obstacle_params[i + 4];
          if (i + 5 < obstacle_params.size() &&
              static_cast<int>(obstacle_params[i + 5]) != 0 &&
              static_cast<int>(obstacle_params[i + 5]) != 1) {
            double height = obstacle_params[i + 5];
            obstacle_centers_.emplace_back(center, radius, height, true);
            i += 6;
          } else {
            obstacle_centers_.emplace_back(center, radius);
            i += 5;
          }
        }
        else if (shape_type == 1 && i + 5 < obstacle_params.size())  // RECTANGLE
        {
          double width = obstacle_params[i + 4];
          double length = obstacle_params[i + 5];
          if (i + 6 < obstacle_params.size() &&
              static_cast<int>(obstacle_params[i + 6]) != 0 &&
              static_cast<int>(obstacle_params[i + 6]) != 1) {
            double height = obstacle_params[i + 6];
            obstacle_centers_.emplace_back(center, width, length, height);
            i += 7;
          } else {
            obstacle_centers_.emplace_back(center, width, length);
            i += 6;
          }
        }
        else
        {
          obstacle_centers_.emplace_back(center);
          i += 3;
        }
      }
      else
      {
        obstacle_centers_.emplace_back(center);
        i += 3;
      }
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load obstacles.yaml, using default obstacles");
    obstacle_centers_ = {
        Obstacle(Eigen::Vector3d(-2.0, -2.25, 0.5)),
        Obstacle(Eigen::Vector3d(1.0, 0.0, 0.5)),
        Obstacle(Eigen::Vector3d(0.0, 1.0, 0.5)),
        Obstacle(Eigen::Vector3d(3.0, 3.0, 0.5))};
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

  // Use dark blue color for optimized path (local trajectory)
  publishPath(optimized_path, drone_id, 0.0f, 0.0f, 1.0f, 0.6, optimized_traj_pub_);
  
  // Only publish obstacles if obstacle avoidance is enabled
  if (enable_obstacles_)
  {
    publishObstacles();
  }

  Eigen::Vector3d current_pos = data.start_pt;
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
    }
  }
  // Handle large jumps by resetting to trajectory start
  if (min_dist > 2.0)
  {
    // RCLCPP_WARN(this->get_logger(), "Drone %d: Large jump found (%.2f m), resetting to trajectory start", drone_id, min_dist);
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

  // Use light blue color for global path (reference, highly visible)
  publishPath(global_path, msg->drone_id, 0.59f, 0.71f, 1.0f, 0.85, global_traj_pub_);
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
            double vx = 0.0, vy = 0.0, vz = 0.0;
            for (int j = 0; j <= data.current_traj.order; ++j)
            {
              double t_pow = std::pow(t_remaining, data.current_traj.order - j);
              x += data.current_traj.coef_x[offset + j] * t_pow;
              y += data.current_traj.coef_y[offset + j] * t_pow;
              z += data.current_traj.coef_z[offset + j] * t_pow;
              // Velocity: derivative of position polynomial
              int exp = data.current_traj.order - j;
              if (exp > 0)
              {
                double t_pow_vel = std::pow(t_remaining, exp - 1);
                vx += data.current_traj.coef_x[offset + j] * exp * t_pow_vel;
                vy += data.current_traj.coef_y[offset + j] * exp * t_pow_vel;
                vz += data.current_traj.coef_z[offset + j] * exp * t_pow_vel;
              }
            }
            data.velocity = Eigen::Vector3d(vx, vy, vz);
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
        data.velocity = Eigen::Vector3d::Zero();
      }
      data.start_pt = Eigen::Vector3d(x, y, z);

      // Accumulate traveled path
      {
        Eigen::Vector3d new_pt(x, y, z);
        auto &tpath = traveled_paths_[drone_id];
        constexpr double kMinPathPointDist = 0.1;
        if (tpath.empty() || (new_pt - tpath.back()).norm() > kMinPathPointDist)
          tpath.push_back(new_pt);
      }
    }

    // Compute arrow direction from velocity; fall back to +x if near-zero
    Eigen::Vector3d vel = data.velocity;
    constexpr double kArrowLength = 1.5;
    constexpr double kArrowShaftDiam = 0.15;
    constexpr double kArrowHeadDiam  = 0.35;
    if (vel.norm() < 1e-3)
      vel = Eigen::Vector3d(1.0, 0.0, 0.0);
    else
      vel.normalize();

    auto [r, g, b] = getDroneColor(drone_id);
    auto marker = createMarker("position_drone_" + std::to_string(drone_id), 0,
                               visualization_msgs::msg::Marker::ARROW, 1.0, r, g, b, 1.0);
    // ARROW with two points: points[0]=tail, points[1]=tip
    marker.scale.x = kArrowShaftDiam;  // shaft diameter
    marker.scale.y = kArrowHeadDiam;   // head diameter
    marker.scale.z = 0.0;
    geometry_msgs::msg::Point tail, tip;
    tail.x = x; tail.y = y; tail.z = z;
    tip.x  = x + vel.x() * kArrowLength;
    tip.y  = y + vel.y() * kArrowLength;
    tip.z  = z + vel.z() * kArrowLength;
    marker.points.push_back(tail);
    marker.points.push_back(tip);
    position_marker_pubs_[drone_id]->publish(marker);
    geometry_msgs::msg::PointStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = this->now();
    pos_msg.point.x = x;
    pos_msg.point.y = y;
    pos_msg.point.z = z;
    position_pubs_[drone_id]->publish(pos_msg);

    // Publish TF frame for follow-camera view (drone_N_base)
    // Frame X-axis aligned with velocity direction for ThirdPersonFollower in RViz
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "drone_" + std::to_string(drone_id) + "_base";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    // Compute yaw and pitch from velocity direction
    double yaw   = std::atan2(vel.y(), vel.x());
    double pitch = -std::asin(std::clamp(vel.z(), -1.0, 1.0));
    tf2::Quaternion q;
    q.setRPY(0.0, pitch, yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void PathVisualization::publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b, float alpha,
                                    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  std::string ns_prefix = (pub == global_traj_pub_) ? "global_path_drone_" : "opt_path_drone_";
  auto marker = createMarker(ns_prefix + std::to_string(id), id,
                             visualization_msgs::msg::Marker::LINE_STRIP, 0.4, r, g, b, alpha);
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
  if (!enable_obstacles_)
  {
    return;
  }

  for (size_t i = 0; i < obstacle_centers_.size(); ++i)
  {
    const auto& obs = obstacle_centers_[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = i;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Marker uses CENTER for its pose; we keep obs.center as the BASE so the
    // column sits on z=center.z. When z_extent > 0, render the true height.
    double z_extent = (obs.z_extent > 0.0) ? obs.z_extent : 2.0;
    marker.pose.position.x = obs.center.x();
    marker.pose.position.y = obs.center.y();
    marker.pose.position.z = obs.center.z() + z_extent * 0.5;
    marker.pose.orientation.w = 1.0;

    if (obs.shape == ObstacleShape::CIRCLE)
    {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      double radius = (obs.param1 > 0) ? obs.param1 : 1.0;
      marker.scale.x = radius * 2.0;
      marker.scale.y = radius * 2.0;
      marker.scale.z = z_extent;
    }
    else if (obs.shape == ObstacleShape::RECTANGLE)
    {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = obs.param1;
      marker.scale.y = obs.param2;
      marker.scale.z = z_extent;
    }

    marker_pub_->publish(marker);
  }

  RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacles", obstacle_centers_.size());
}


void PathVisualization::publishTraveledPaths()
{
  for (int drone_id = 0; drone_id < num_drones_; ++drone_id)
  {
    const auto &path = traveled_paths_[drone_id];
    if (path.size() < 2)
      continue;

    auto [r, g, b] = getDroneColor(drone_id);
    auto marker = createMarker("traveled_path_drone_" + std::to_string(drone_id), drone_id,
                               visualization_msgs::msg::Marker::LINE_STRIP, 0.1, r, g, b, 0.85);
    marker.lifetime = rclcpp::Duration(0, 0); // Never expire in RViz

    for (const auto &pt : path)
    {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      marker.points.push_back(p);
    }
    traveled_path_pubs_[drone_id]->publish(marker);
  }
}

void PathVisualization::loadRiskZoneParameters()
{
  this->declare_parameter("risk_zones", std::vector<double>{});
  std::vector<double> tz_params;
  this->get_parameter("risk_zones", tz_params);
  RCLCPP_INFO(this->get_logger(), "Risk zone params size: %zu", tz_params.size());
  if (tz_params.size() >= 5 && tz_params.size() % 5 == 0) {
    for (size_t i = 0; i < tz_params.size(); i += 5) {
      VisRiskZone tz;
      tz.center = Eigen::Vector3d(tz_params[i], tz_params[i+1], tz_params[i+2]);
      tz.reach = tz_params[i+3];
      tz.peak = tz_params[i+4];
      risk_zones_.push_back(tz);
      RCLCPP_INFO(this->get_logger(), "  RiskZone #%zu: center=(%.1f,%.1f,%.1f) range=%.1f risk=%.1f",
          risk_zones_.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
          tz.reach, tz.peak);
    }
  } else if (!tz_params.empty()) {
    RCLCPP_WARN(this->get_logger(), "Invalid risk_zones param size: %zu (must be multiple of 5)", tz_params.size());
  }
}

void PathVisualization::publishRiskZones()
{
  if (!risk_zone_pub_) return;
  // ADD-only publish: every active zone gets a fresh stamp every 2 s, so
  // the markers stay alive without flicker. DELETEALL is only sent when
  // the zone list actually changes (in riskZoneArrayCallback) — never
  // here on the periodic timer.
  if (risk_zones_.empty()) return;

  int marker_id = 0;
  for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
    const auto &tz = risk_zones_[zi];

    // Single risk sphere (sensing range, red semi-transparent).
    // Represents the unified risk volume — danger decays smoothly from center.
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.ns = "risk_zone";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = tz.center.x();
      m.pose.position.y = tz.center.y();
      m.pose.position.z = tz.center.z();
      m.pose.orientation.w = 1.0;
      double d = tz.reach * 2.0;
      m.scale.x = d; m.scale.y = d; m.scale.z = d;
      // V3: peak in (0, 1]. Map to alpha in [0.1, 0.5] for visibility.
      float alpha = 0.1f + 0.4f * static_cast<float>(tz.peak);
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = alpha;
      m.lifetime = rclcpp::Duration(0, 0);
      risk_zone_pub_->publish(m);
    }

    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.ns = "risk_center";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = tz.center.x();
      m.pose.position.y = tz.center.y();
      m.pose.position.z = tz.center.z();
      m.pose.orientation.w = 1.0;
      m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 0.5;
      m.color.r = 0.8; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
      m.lifetime = rclcpp::Duration(0, 0);
      risk_zone_pub_->publish(m);
    }

    // Text label
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.ns = "risk_label";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = tz.center.x();
      m.pose.position.y = tz.center.y();
      m.pose.position.z = tz.center.z() + tz.reach + 1.0;
      m.pose.orientation.w = 1.0;
      m.scale.z = 1.5;
      m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0;
      std::ostringstream ss;
      ss << "restricted zone #" << zi << " (peak=" << std::fixed << std::setprecision(2) << tz.peak << ")";
      m.text = ss.str();
      m.lifetime = rclcpp::Duration(0, 0);
      risk_zone_pub_->publish(m);
    }
  }
}
