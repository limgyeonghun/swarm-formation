#include <rclcpp/rclcpp.hpp>
#include "formation_msgs/msg/trajectory_command.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "formation_manager/formation_utils.h"
#include "formation_manager/hungarian_algorithm.h"
#include "swarm_graph/swarm_graph.hpp"

using namespace std::chrono_literals;

// Structure to hold mission command data from YAML
struct MissionCommand {
    std::string formation_type;
    double formation_scale;
    double distance_threshold;
    double formation_similarity_threshold;
    std::vector<std::vector<double>> waypoints;
    std::vector<Eigen::Vector3d> start_positions;  // Start positions for each drone
};

/**
 * @brief FormationManager - manages formation changes and publishes individual trajectory commands
 *
 * Key differences from old FormationCommander:
 * - Publishes TrajectoryCommand (individual targets) instead of FormationCommand (group command)
 * - Each drone gets its own personal target and waypoints
 * - Handles Hungarian assignment internally
 * - trajectory_planner nodes receive ready-to-use targets
 */
class FormationManager : public rclcpp::Node
{
public:
    FormationManager()
    : Node("formation_manager"),
      mission_sequence_(0),
      current_formation_center_(0.0, 0.0, 0.0),
      previous_formation_type_("")
    {
        // Parameters
        this->declare_parameter("num_drones", 4);
        this->declare_parameter("distance_threshold", 3.0);
        this->declare_parameter("formation_similarity_threshold", 2.0);
        this->declare_parameter("formation_z_spacing", 2.0);
        this->declare_parameter("scenario", "default");

        num_drones_ = this->get_parameter("num_drones").as_int();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        formation_similarity_threshold_ = this->get_parameter("formation_similarity_threshold").as_double();
        formation_z_spacing_ = this->get_parameter("formation_z_spacing").as_double();
        scenario_ = this->get_parameter("scenario").as_string();

        // Load missions from YAML
        loadMissionFromYAML();

        RCLCPP_INFO(this->get_logger(),
                   "FormationManager started - num_drones: %d, loaded %zu missions",
                   num_drones_, missions_.size());

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Create publishers for each drone
        for (int i = 0; i < num_drones_; ++i) {
            std::string topic_name = "/V" + std::to_string(i + 1) + "/trajectory_command";
            auto pub = this->create_publisher<formation_msgs::msg::TrajectoryCommand>(topic_name, sensor_qos);
            trajectory_cmd_pubs_.push_back(pub);
        }

        // Subscribe to odometry from all drones (for position tracking)
        for (int i = 0; i < num_drones_; ++i) {
            std::string odom_topic = "/V" + std::to_string(i + 1) + "/odom";
            auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, sensor_qos,
                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    this->odometryCallback(msg, i);
                }
            );
            odometry_subs_.push_back(sub);
        }

        drone_positions_.resize(num_drones_, Eigen::Vector3d::Zero());
        positions_valid_.resize(num_drones_, false);

        // Initialize SwarmGraph
        swarm_graph_ = std::make_unique<SwarmGraph>();

        // Publish first mission after 2 seconds
        initial_timer_ = this->create_wall_timer(
            2000ms,
            [this]() {
                publishTrajectoryCommands();
                initial_timer_->cancel();

                // Start distance checking timer
                distance_check_timer_ = this->create_wall_timer(
                    100ms,
                    std::bind(&FormationManager::checkDistanceAndPublish, this)
                );

                RCLCPP_INFO(this->get_logger(),
                    "Published initial mission, switched to distance-based mode (threshold: %.1f m)",
                    distance_threshold_);
            }
        );

        RCLCPP_INFO(this->get_logger(),
                   "FormationManager initialized - will publish first mission in ~2s");
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int drone_idx) {
        if (drone_idx < 0 || drone_idx >= num_drones_) return;

        drone_positions_[drone_idx] = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        positions_valid_[drone_idx] = true;
    }

    void loadMissionFromYAML() {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("formation_manager");
            std::string yaml_file = package_share_dir + "/config/scenario_" + scenario_ + ".yaml";

            RCLCPP_INFO(this->get_logger(), "Loading scenario: %s", yaml_file.c_str());
            YAML::Node config = YAML::LoadFile(yaml_file);

            // Load start positions for each drone
            std::vector<Eigen::Vector3d> start_positions(num_drones_, Eigen::Vector3d::Zero());
            for (int i = 0; i < num_drones_; ++i) {
                std::string drone_key = "drone_" + std::to_string(i);
                if (config[drone_key]) {
                    double x = config[drone_key]["start_point_x"].as<double>(0.0);
                    double y = config[drone_key]["start_point_y"].as<double>(0.0);
                    double z = config[drone_key]["start_point_z"].as<double>(0.0);
                    start_positions[i] = Eigen::Vector3d(x, y, z);
                    RCLCPP_INFO(this->get_logger(), "Loaded start position for drone_%d: (%.2f, %.2f, %.2f)",
                                i, x, y, z);
                }
            }

            if (config["scenario"] && config["scenario"]["mission"] && config["scenario"]["mission"]["commands"]) {
                YAML::Node commands = config["scenario"]["mission"]["commands"];

                for (const auto& cmd_node : commands) {
                    MissionCommand cmd;
                    cmd.formation_type = cmd_node["formation_type"].as<std::string>();
                    cmd.formation_scale = cmd_node["formation_scale"].as<double>();
                    cmd.distance_threshold = cmd_node["distance_threshold"].as<double>();
                    cmd.formation_similarity_threshold = cmd_node["formation_similarity_threshold"] ?
                        cmd_node["formation_similarity_threshold"].as<double>() : -1.0;

                    if (cmd_node["waypoints"]) {
                        for (const auto& wp : cmd_node["waypoints"]) {
                            std::vector<double> waypoint = {
                                wp[0].as<double>(),
                                wp[1].as<double>(),
                                wp[2].as<double>()
                            };
                            cmd.waypoints.push_back(waypoint);
                        }
                    }

                    // Assign start positions to this mission
                    cmd.start_positions = start_positions;

                    missions_.push_back(cmd);
                }

                RCLCPP_INFO(this->get_logger(), "Loaded %zu missions from YAML", missions_.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
        }
    }

    void checkDistanceAndPublish() {
        if (mission_sequence_ >= (int)missions_.size()) {
            return;  // All missions completed
        }

        // Get current swarm center
        Eigen::Vector3d current_center = getCurrentSwarmCenter();
        double distance = (current_center - current_formation_center_).norm();

        const auto& current_mission = missions_[mission_sequence_];
        double threshold = current_mission.distance_threshold;

        bool distance_check_enabled = (threshold >= 0.0);
        bool distance_condition_met = distance_check_enabled && (distance <= threshold);

        // Check formation similarity
        bool similarity_condition_met = false;
        if (current_mission.formation_similarity_threshold >= 0.0 && !previous_formation_type_.empty()) {
            if (previous_formation_type_ != current_mission.formation_type) {
                double similarity = calculateFormationSimilarity();
                similarity_condition_met = (similarity <= current_mission.formation_similarity_threshold);
            }
        }

        bool should_publish = false;
        if (!distance_check_enabled) {
            should_publish = similarity_condition_met;
        } else {
            should_publish = (distance_condition_met || similarity_condition_met);
        }

        if (should_publish) {
            mission_sequence_++;
            if (mission_sequence_ < (int)missions_.size()) {
                publishTrajectoryCommands();
                RCLCPP_INFO(this->get_logger(), "Published mission %d/%zu",
                           mission_sequence_ + 1, missions_.size());
            }
        }
    }

    Eigen::Vector3d getCurrentSwarmCenter() {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        int valid_count = 0;

        for (int i = 0; i < num_drones_; ++i) {
            if (positions_valid_[i]) {
                center += drone_positions_[i];
                valid_count++;
            }
        }

        if (valid_count > 0) {
            center /= static_cast<double>(valid_count);
        }

        return center;
    }

    double calculateFormationSimilarity() {
        std::vector<Eigen::Vector3d> current_positions;
        for (int i = 0; i < num_drones_; ++i) {
            if (positions_valid_[i]) {
                current_positions.push_back(drone_positions_[i]);
            }
        }

        if (current_positions.size() != static_cast<size_t>(num_drones_)) {
            return std::numeric_limits<double>::infinity();
        }

        const auto& mission = missions_[mission_sequence_];
        std::vector<Eigen::Vector3d> target_pattern =
            formation_manager::FormationUtils::generateFormationPattern(
                mission.formation_type, num_drones_, mission.formation_scale, formation_z_spacing_);

        swarm_graph_->setDesiredForm(target_pattern);
        swarm_graph_->updateGraph(current_positions);

        double similarity_cost;
        if (!swarm_graph_->calcFNorm2(similarity_cost)) {
            return std::numeric_limits<double>::infinity();
        }

        return similarity_cost;
    }

    void publishTrajectoryCommands() {
        if (mission_sequence_ >= (int)missions_.size()) {
            RCLCPP_WARN(this->get_logger(), "No more missions to publish");
            return;
        }

        const auto& mission = missions_[mission_sequence_];

        // Calculate formation center (last waypoint)
        if (!mission.waypoints.empty()) {
            const auto& last_wp = mission.waypoints.back();
            current_formation_center_ = Eigen::Vector3d(last_wp[0], last_wp[1], last_wp[2]);
        }

        // Generate formation pattern (relative positions)
        std::vector<Eigen::Vector3d> formation_pattern =
            formation_manager::FormationUtils::generateFormationPattern(
                mission.formation_type, num_drones_, mission.formation_scale, formation_z_spacing_);

        // Get current drone positions
        std::vector<Eigen::Vector3d> current_positions;
        for (int i = 0; i < num_drones_; ++i) {
            if (positions_valid_[i]) {
                current_positions.push_back(drone_positions_[i]);
            } else {
                current_positions.push_back(Eigen::Vector3d::Zero());
            }
        }

        // Generate target positions (formation center + pattern)
        std::vector<Eigen::Vector3d> target_positions(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            target_positions[i] = current_formation_center_ + formation_pattern[i];
        }

        // Compute Hungarian assignment
        std::vector<int> assignment = computeHungarianAssignment(
            current_positions, target_positions, mission.formation_type);

        // Publish individual TrajectoryCommand for each drone
        for (int i = 0; i < num_drones_; ++i) {
            auto msg = formation_msgs::msg::TrajectoryCommand();
            msg.header.stamp = this->now();
            msg.drone_id = i;
            msg.sequence = mission_sequence_;
            msg.mission_id = "mission_" + std::to_string(mission_sequence_);

            // Set start position from mission data
            if (i < (int)mission.start_positions.size()) {
                msg.start_position.x = mission.start_positions[i].x();
                msg.start_position.y = mission.start_positions[i].y();
                msg.start_position.z = mission.start_positions[i].z();
            } else {
                // Fallback to zero if not available
                msg.start_position.x = 0.0;
                msg.start_position.y = 0.0;
                msg.start_position.z = 0.0;
            }

            // Assign target based on Hungarian result
            int target_idx = assignment[i];
            Eigen::Vector3d my_target = target_positions[target_idx];

            msg.target_position.x = my_target.x();
            msg.target_position.y = my_target.y();
            msg.target_position.z = my_target.z();

            msg.target_velocity.x = 0.0;
            msg.target_velocity.y = 0.0;
            msg.target_velocity.z = 0.0;

            // Waypoints (same for all drones, will be used for global path)
            msg.waypoints.resize(mission.waypoints.size());
            for (size_t j = 0; j < mission.waypoints.size(); ++j) {
                msg.waypoints[j].x = mission.waypoints[j][0];
                msg.waypoints[j].y = mission.waypoints[j][1];
                msg.waypoints[j].z = mission.waypoints[j][2];
            }

            msg.formation_type = mission.formation_type;
            msg.formation_scale = mission.formation_scale;
            msg.formation_offset.x = formation_pattern[target_idx].x();
            msg.formation_offset.y = formation_pattern[target_idx].y();
            msg.formation_offset.z = formation_pattern[target_idx].z();

            // Add full formation pattern for optimizer
            msg.formation_pattern.resize(formation_pattern.size());
            for (size_t j = 0; j < formation_pattern.size(); ++j) {
                msg.formation_pattern[j].x = formation_pattern[j].x();
                msg.formation_pattern[j].y = formation_pattern[j].y();
                msg.formation_pattern[j].z = formation_pattern[j].z();
            }

            trajectory_cmd_pubs_[i]->publish(msg);
        }

        previous_formation_type_ = mission.formation_type;

        RCLCPP_INFO(this->get_logger(),
                   "Published mission %d: %s (scale: %.1f) to %d drones",
                   mission_sequence_, mission.formation_type.c_str(),
                   mission.formation_scale, num_drones_);
    }

    std::vector<int> computeHungarianAssignment(
        const std::vector<Eigen::Vector3d>& current_positions,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::string& formation_type)
    {
        int n = num_drones_;
        std::vector<int> assignment(n);

        // For line formations, preserve order
        if (formation_type.find("line") != std::string::npos) {
            std::iota(assignment.begin(), assignment.end(), 0);
            return assignment;
        }

        // For other formations, use Hungarian algorithm
        formation_manager::HungarianAlgorithm hungarian;
        std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n));

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                cost_matrix[i][j] = (current_positions[i] - target_positions[j]).norm();
            }
        }

        assignment = hungarian.solve(cost_matrix);
        return assignment;
    }

    int num_drones_;
    double distance_threshold_;
    double formation_similarity_threshold_;
    double formation_z_spacing_;
    int mission_sequence_;
    std::string scenario_;

    std::vector<MissionCommand> missions_;
    std::vector<rclcpp::Publisher<formation_msgs::msg::TrajectoryCommand>::SharedPtr> trajectory_cmd_pubs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odometry_subs_;

    std::vector<Eigen::Vector3d> drone_positions_;
    std::vector<bool> positions_valid_;

    Eigen::Vector3d current_formation_center_;
    std::string previous_formation_type_;

    rclcpp::TimerBase::SharedPtr initial_timer_;
    rclcpp::TimerBase::SharedPtr distance_check_timer_;

    SwarmGraph::Ptr swarm_graph_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormationManager>());
    rclcpp::shutdown();
    return 0;
}
