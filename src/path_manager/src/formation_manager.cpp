#include "path_manager/formation_manager.h"
#include <cmath>

namespace path_manager
{

FormationManager::FormationManager() 
  : Node("formation_manager"),
    num_drones_(4),
    current_formation_type_("square"),
    current_formation_scale_(2.0),
    has_formation_command_(false),
    all_positions_received_(false)
{
  // Declare parameters
  this->declare_parameter("num_drones", 4);
  this->declare_parameter("formation_type", "square");
  this->declare_parameter("formation_scale", 2.0);
  this->declare_parameter("formation_center_x", 0.0);
  this->declare_parameter("formation_center_y", 80.0);
  this->declare_parameter("formation_center_z", 0.0);

  // Get parameters
  this->get_parameter("num_drones", num_drones_);
  this->get_parameter("formation_type", current_formation_type_);
  this->get_parameter("formation_scale", current_formation_scale_);
  
  double center_x, center_y, center_z;
  this->get_parameter("formation_center_x", center_x);
  this->get_parameter("formation_center_y", center_y);
  this->get_parameter("formation_center_z", center_z);
  current_formation_center_ = Eigen::Vector3d(center_x, center_y, center_z);

  // Initialize SwarmGraph
  swarm_graph_ = std::make_unique<SwarmGraph>();
  current_positions_.resize(num_drones_);
  position_received_.resize(num_drones_, false);
  drone_pose_subs_.resize(num_drones_);

  // Set up QoS profile
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  // Create subscribers and publishers
  formation_cmd_sub_ = this->create_subscription<path_manager::msg::FormationCommand>(
    "formation_command", sensor_qos, 
    std::bind(&FormationManager::formationCommandCallback, this, std::placeholders::_1));

  formation_target_pub_ = this->create_publisher<path_manager::msg::FormationTarget>(
    "formation_targets", sensor_qos);

  // Create subscribers for each drone's position
  for (int i = 0; i < num_drones_; ++i) {
    std::string topic_name = "/drone_" + std::to_string(i) + "/pose";
    drone_pose_subs_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_name, sensor_qos,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->dronePositionCallback(msg, i);
      });
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic_name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "FormationManager initialized with %d drones", num_drones_);
  RCLCPP_INFO(this->get_logger(), "Initial formation: %s, scale: %.2f",
              current_formation_type_.c_str(), current_formation_scale_);
  RCLCPP_INFO(this->get_logger(), "Initial center: (%.2f, %.2f, %.2f)",
              current_formation_center_.x(),
              current_formation_center_.y(),
              current_formation_center_.z());
}

void FormationManager::dronePositionCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg, int drone_id)
{
  if (drone_id >= 0 && drone_id < num_drones_) {
    current_positions_[drone_id] = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y, 
      msg->pose.position.z
    );

    if (!position_received_[drone_id]) {
      position_received_[drone_id] = true;
      RCLCPP_INFO(this->get_logger(),
                  "Received first position for drone %d: (%.2f, %.2f, %.2f)", 
                  drone_id,
                  msg->pose.position.x,
                  msg->pose.position.y,
                  msg->pose.position.z);
    }

    // Check if all positions are received
    all_positions_received_ = true;
    for (bool received : position_received_) {
      if (!received) {
        all_positions_received_ = false;
        break;
      }
    }

    if (all_positions_received_) {
      updateSwarmGraph();
    }
  }
}

void FormationManager::updateSwarmGraph()
{
  if (swarm_graph_ && all_positions_received_) {
    if (!swarm_graph_->updateGraph(current_positions_)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "SwarmGraph update skipped - desired formation not ready");
    } else {
      RCLCPP_DEBUG(this->get_logger(),
                   "Successfully updated SwarmGraph with current positions");
    }
  }
}

void FormationManager::formationCommandCallback(
  const path_manager::msg::FormationCommand::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
              "Received formation command: %s, scale: %.2f, center: (%.2f, %.2f, %.2f)",
              msg->formation_type.c_str(),
              msg->formation_scale,
              msg->formation_center.x,
              msg->formation_center.y,
              msg->formation_center.z);

  // Update formation parameters
  current_formation_type_ = msg->formation_type;
  current_formation_scale_ = msg->formation_scale;
  current_formation_center_ = Eigen::Vector3d(
    msg->formation_center.x, 
    msg->formation_center.y, 
    msg->formation_center.z
  );
  
  has_formation_command_ = true;

  // Generate and publish new formation targets
  generateFormationTargets(current_formation_center_,
                           current_formation_type_,
                           current_formation_scale_);
}

void FormationManager::generateFormationTargets(
  const Eigen::Vector3d& center, 
  const std::string& formation_type, 
  double scale)
{
  // Generate formation pattern
  std::vector<Eigen::Vector3d> formation_pattern =
    generateFormationPattern(formation_type, num_drones_, scale);

  // Translate pattern to center position
  std::vector<Eigen::Vector3d> targets;
  targets.reserve(num_drones_);

  for (const auto& pattern_point : formation_pattern) {
    targets.push_back(center + pattern_point);
  }

  // Set desired formation in SwarmGraph
  if (swarm_graph_) {
    swarm_graph_->setDesiredForm(targets);
    RCLCPP_INFO(this->get_logger(), "Set desired formation in SwarmGraph: %s", formation_type.c_str());
    
    // Update SwarmGraph with current positions if available
    if (all_positions_received_) {
      updateSwarmGraph();
    }
  }

  // Publish formation targets
  publishFormationTargets(targets);
}

std::vector<Eigen::Vector3d> FormationManager::generateFormationPattern(
  const std::string& formation_type, int num_drones, double scale)
{
  std::vector<Eigen::Vector3d> pattern;
  pattern.reserve(num_drones);

  if (formation_type == "square" && num_drones == 4) {
    // Square formation for 4 drones
    pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));
    pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));
    pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));
    pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, 0.0));
  }
  else if (formation_type == "triangle" && num_drones >= 3) {
    // Triangle formation
    double angle_step = 2.0 * M_PI / 3.0;
    for (int i = 0; i < 3 && i < num_drones; ++i) {
      double angle = i * angle_step;
      pattern.push_back(Eigen::Vector3d(
        scale * cos(angle), 
        scale * sin(angle), 
        0.0
      ));
    }
    for (int i = 3; i < num_drones; ++i) {
      pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    }
  }
  else if (formation_type == "line") {
    // Line formation
    double spacing = scale / (num_drones - 1);
    for (int i = 0; i < num_drones; ++i) {
      pattern.push_back(Eigen::Vector3d(
        -scale/2 + i * spacing, 
        0.0, 
        0.0
      ));
    }
  }
  else if (formation_type == "circle") {
    // Circle formation
    double angle_step = 2.0 * M_PI / num_drones;
    for (int i = 0; i < num_drones; ++i) {
      double angle = i * angle_step;
      pattern.push_back(Eigen::Vector3d(
        scale * cos(angle), 
        scale * sin(angle), 
        0.0
      ));
    }
  }
  else {
    // Default: square formation (fallback)
    RCLCPP_INFO(this->get_logger(),
                "Unknown formation type '%s', using square formation",
                formation_type.c_str());

    if (num_drones <= 4) {
      pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));
      if (num_drones > 1) pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));
      if (num_drones > 2) pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, 0.0));
      if (num_drones > 3) pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));
    } else {
      double angle_step = 2.0 * M_PI / num_drones;
      for (int i = 0; i < num_drones; ++i) {
        double angle = i * angle_step;
        pattern.push_back(Eigen::Vector3d(
          scale * cos(angle), 
          scale * sin(angle), 
          0.0
        ));
      }
    }
  }

  return pattern;
}

void FormationManager::publishFormationTargets(
  const std::vector<Eigen::Vector3d>& targets)
{
  auto current_time = this->now();

  for (int i = 0; i < num_drones_ && i < static_cast<int>(targets.size()); ++i) {
    path_manager::msg::FormationTarget target_msg;

    target_msg.header.stamp = current_time;
    target_msg.header.frame_id = "world";
    target_msg.drone_id = i;

    target_msg.target_position.x = targets[i].x();
    target_msg.target_position.y = targets[i].y();
    target_msg.target_position.z = targets[i].z();

    target_msg.target_velocity.x = 0.0;
    target_msg.target_velocity.y = 0.0;
    target_msg.target_velocity.z = 0.0;

    // Add formation information
    target_msg.formation_type = current_formation_type_;
    target_msg.formation_scale = current_formation_scale_;
    
    // Add all formation positions
    target_msg.formation_positions.clear();
    target_msg.formation_positions.reserve(targets.size());
    for (const auto& target : targets) {
      geometry_msgs::msg::Point formation_point;
      formation_point.x = target.x();
      formation_point.y = target.y();
      formation_point.z = target.z();
      target_msg.formation_positions.push_back(formation_point);
    }

    formation_target_pub_->publish(target_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published target for drone %d: (%.2f, %.2f, %.2f)", 
                 i,
                 targets[i].x(),
                 targets[i].y(),
                 targets[i].z());
  }

  RCLCPP_INFO(this->get_logger(),
              "Published formation targets for %d drones", num_drones_);
}

} // namespace path_manager
