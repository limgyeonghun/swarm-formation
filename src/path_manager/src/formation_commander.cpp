#include <rclcpp/rclcpp.hpp>
#include "path_manager/msg/formation_command.hpp"
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "path_optimizer/poly_traj_utils.hpp"
#include "path_manager/hungarian_algorithm.h"
#include "path_manager/formation_utils.h"
#include "swarm_graph/swarm_graph.hpp"

using namespace std::chrono_literals;

// Structure to hold formation command data from YAML
struct FormationCommand {
    std::string formation_type;
    double formation_scale;
    double distance_threshold;
    double formation_similarity_threshold;  // RMSE threshold for same formation type
    std::vector<std::vector<double>> waypoints;
};

/**
 * @brief Lightweight FormationCommander - only publishes formation commands
 *
 * Task assignment is now handled in a distributed manner by each drone's replan_fsm
 */
class FormationCommander : public rclcpp::Node
{
public:
    FormationCommander()
    : Node("formation_commander"),
      command_count_(0),
      current_formation_center_(0.0, 0.0, 0.0),
      have_initial_command_(false),
      previous_formation_type_(""),
      mission_sequence_(0),
      last_published_command_count_(-1)
    {
        // Declare parameters
        this->declare_parameter("num_drones", 4);
        this->declare_parameter("distance_threshold", 3.0);
        this->declare_parameter("formation_similarity_threshold", 2.0);
        this->declare_parameter("formation_z_spacing", 2.0);

        // Get parameters
        num_drones_ = this->get_parameter("num_drones").as_int();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        formation_similarity_threshold_ = this->get_parameter("formation_similarity_threshold").as_double();
        formation_z_spacing_ = this->get_parameter("formation_z_spacing").as_double();

        // Load mission from YAML (mission.commands)
        loadMissionFromYAML();

        RCLCPP_INFO(this->get_logger(), "FormationCommander started - num_drones: %d, distance_threshold: %.1f, loaded %zu commands",
                   num_drones_, distance_threshold_, current_scenario_.size());

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 10),
            qos_profile
        );

        formation_cmd_pub_ = this->create_publisher<path_manager::msg::FormationCommand>(
            "formation_command", sensor_qos
        );

        // Subscribe to verified trajectories from FSM1 (all drones)
        // FSM1 publishes both its own trajectory and received trajectories after validation
        auto traj_sub = this->create_subscription<path_manager::msg::PolyTraj>(
            "/for_commander/trajectories", sensor_qos,
            [this](const path_manager::msg::PolyTraj::SharedPtr msg) {
                this->trajectoryCallback(msg, msg->drone_id);
            }
        );
        trajectory_subs_.push_back(traj_sub);

        drone_trajectories_.resize(num_drones_);  // Track all drones

        // Publish first command after 2 seconds, then switch to distance-based
        initial_timer_ = this->create_wall_timer(
            2000ms,
            [this]() {
                publishFormationCommand();
                initial_timer_->cancel();
                have_initial_command_ = true;

                distance_check_timer_ = this->create_wall_timer(
                    100ms,
                    std::bind(&FormationCommander::checkDistanceAndPublish, this)
                );

                RCLCPP_INFO(this->get_logger(),
                    "Switched to distance-based formation commands (threshold: %.1f m)", distance_threshold_);
            }
        );

        // Periodic republishing timer (1Hz) for robustness
        // Continuously publishes the current mission so FSMs can recover if they miss a message
        periodic_pub_timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&FormationCommander::periodicRepublish, this)
        );

        // Initialize SwarmGraph
        swarm_graph_ = std::make_unique<SwarmGraph>();

        RCLCPP_INFO(
            this->get_logger(),
            "FormationCommander started - will publish first command in ~2s, then distance-based"
        );
    }

private:
    struct TrajectoryData {
        poly_traj::Trajectory traj;
        double start_time;
        double duration;
        int drone_id;
        bool valid;

        TrajectoryData() : start_time(0.0), duration(0.0), drone_id(-1), valid(false) {}
    };

    void trajectoryCallback(const path_manager::msg::PolyTraj::SharedPtr msg, int drone_idx) {
        if (drone_idx < 0 || drone_idx >= num_drones_) return;

        if (msg->drone_id != drone_idx) {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring trajectory: expected drone_id=%d, got drone_id=%d",
                        drone_idx, msg->drone_id);
            return;
        }

        auto& traj_data = drone_trajectories_[drone_idx];
        traj_data.drone_id = msg->drone_id;
        traj_data.start_time = rclcpp::Time(msg->start_time).seconds();
        traj_data.valid = true;

        int piece_nums = msg->duration.size();
        std::vector<double> dura(piece_nums);
        std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
        for (int i = 0; i < piece_nums; ++i) {
            int i6 = i * 6;
            cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                               msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
            cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                               msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
            cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                               msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];
            dura[i] = msg->duration[i];
        }

        traj_data.traj = poly_traj::Trajectory(dura, cMats);
        traj_data.duration = traj_data.traj.getTotalDuration();
    }

    void checkDistanceAndPublish() {
        if (!have_initial_command_ || command_count_ >= (int)current_scenario_.size()) {
            return;
        }

        Eigen::Vector3d current_swarm_center = getCurrentSwarmCenter();
        Eigen::Vector3d target_center = getTargetFormationCenter();
        double distance = (current_swarm_center - target_center).norm();

        // Use the current command's distance_threshold
        double current_threshold = (command_count_ > 0 && command_count_ <= (int)current_scenario_.size())
                                   ? current_scenario_[command_count_ - 1].distance_threshold
                                   : distance_threshold_;

        // If distance_threshold is -1, disable distance check
        bool distance_check_enabled = (current_threshold >= 0.0);
        bool distance_condition_met = distance_check_enabled && (distance <= current_threshold);

        // Check formation similarity when transitioning FROM previous TO current formation
        // Compare PREVIOUS formation with CURRENT formation (not current with next)
        bool similarity_condition_met = false;
        if (command_count_ > 0 && command_count_ <= (int)current_scenario_.size()) {
            const auto& current_cmd = current_scenario_[command_count_ - 1];
            double sim_threshold = current_cmd.formation_similarity_threshold;

            // Only check similarity if threshold is specified (>= 0)
            if (sim_threshold >= 0.0) {
                // Check if transitioning from different formation type
                bool formation_transition = false;
                bool current_is_line = false;

                // Compare previous formation type with current formation type
                if (!previous_formation_type_.empty()) {
                    formation_transition = (previous_formation_type_ != current_cmd.formation_type);

                    // Exclude if CURRENT formation is line (but allow LINE -> SQUARE transition)
                    current_is_line = (current_cmd.formation_type.find("line") != std::string::npos);
                }
                
                if (formation_transition && !current_is_line) {
                    double similarity = calculateFormationSimilarity();
                    similarity_condition_met = (similarity <= sim_threshold);

                    // Similarity condition met - log removed to reduce terminal output
                }
            }
        }

        // Publish next command based on conditions
        // If distance_threshold is -1, only use similarity condition
        bool should_publish = false;
        if (!distance_check_enabled) {
            // distance_threshold is -1: only use similarity
            should_publish = similarity_condition_met;
            // Distance check disabled - log removed to reduce terminal output
        } else {
            // Normal mode: either condition met
            should_publish = (distance_condition_met || similarity_condition_met);
            if (should_publish && distance_condition_met) {
                RCLCPP_INFO(this->get_logger(),
                           "Distance threshold reached (%.2f <= %.2f), publishing next formation command",
                           distance, current_threshold);
            }
        }

        if (should_publish) {
            publishFormationCommand();
        }
    }

    Eigen::Vector3d getCurrentSwarmCenter() {
        double current_time = this->now().seconds();
        Eigen::Vector3d center(0, 0, 0);
        int valid_drones = 0;

        for (int i = 0; i < num_drones_; ++i) {
            const auto& traj = drone_trajectories_[i];
            if (traj.valid) {
                double t_rel = current_time - traj.start_time;
                t_rel = std::min(traj.duration, std::max(0.0, t_rel));
                Eigen::Vector3d pos = traj.traj.getPos(t_rel);
                center += pos;
                valid_drones++;
            }
        }

        if (valid_drones > 0) {
            center /= valid_drones;
        }

        return center;
    }

    Eigen::Vector3d getTargetFormationCenter() {
        return current_formation_center_;
    }

    // Get current drone positions from trajectories
    std::vector<Eigen::Vector3d> getCurrentDronePositions() {
        double current_time = this->now().seconds();
        std::vector<Eigen::Vector3d> positions(num_drones_);

        for (int i = 0; i < num_drones_; ++i) {
            const auto& traj = drone_trajectories_[i];
            if (traj.valid) {
                double t_rel = current_time - traj.start_time;
                t_rel = std::min(traj.duration, std::max(0.0, t_rel));
                positions[i] = traj.traj.getPos(t_rel);
            } else {
                // Fallback: use (0,0,0) if trajectory not available
                positions[i] = Eigen::Vector3d(0, 0, 0);
                RCLCPP_WARN(this->get_logger(), "Drone %d trajectory not valid, using fallback position", i);
            }
        }

        return positions;
    }

    // Generate target formation positions based on current command
    std::vector<Eigen::Vector3d> generateTargetFormationPositions() {
        if (command_count_ <= 0 || command_count_ > (int)current_scenario_.size()) {
            RCLCPP_WARN(this->get_logger(), "Invalid command_count_ in generateTargetFormationPositions");
            return std::vector<Eigen::Vector3d>(num_drones_, Eigen::Vector3d(0, 0, 0));
        }

        const auto& current_cmd = current_scenario_[command_count_ - 1];
        std::string formation_type = current_cmd.formation_type;
        double formation_scale = current_cmd.formation_scale;

        // Generate formation pattern (relative positions)
        std::vector<Eigen::Vector3d> pattern = path_manager::FormationUtils::generateFormationPattern(
            formation_type, num_drones_, formation_scale, formation_z_spacing_);

        // Translate pattern to current formation center
        std::vector<Eigen::Vector3d> target_positions(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            target_positions[i] = pattern[i] + current_formation_center_;
        }

        return target_positions;
    }

    // Calculate formation similarity using Laplacian-based method (from SwarmGraph)
    // This compares the geometric structure of the formation using Normalized Laplacian matrices
    // Returns: Frobenius norm squared ||L_current - L_desired||²_F
    double calculateFormationSimilarity() {
        std::vector<Eigen::Vector3d> current_positions = getCurrentDronePositions();

        if (command_count_ <= 0 || command_count_ > (int)current_scenario_.size()) {
            RCLCPP_WARN(this->get_logger(), "Invalid command_count_ in calculateFormationSimilarity");
            return std::numeric_limits<double>::infinity();
        }

        const auto& current_cmd = current_scenario_[command_count_ - 1];
        std::string formation_type = current_cmd.formation_type;
        double formation_scale = current_cmd.formation_scale;

        // Generate target formation pattern (relative positions from origin)
        std::vector<Eigen::Vector3d> target_pattern = path_manager::FormationUtils::generateFormationPattern(
            formation_type, num_drones_, formation_scale, formation_z_spacing_);

        if (current_positions.size() != target_pattern.size()) {
            RCLCPP_ERROR(this->get_logger(), "Size mismatch in calculateFormationSimilarity");
            return std::numeric_limits<double>::infinity();
        }

        // Update SwarmGraph with current positions
        if (!swarm_graph_->updateGraph(current_positions)) {
            // If update fails (desired not set yet), set desired and try again
            swarm_graph_->setDesiredForm(target_pattern);
            if (!swarm_graph_->updateGraph(current_positions)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update SwarmGraph");
                return std::numeric_limits<double>::infinity();
            }
        }

        // Set desired formation (target pattern)
        swarm_graph_->setDesiredForm(target_pattern);

        // Update graph again with current positions now that desired is set
        swarm_graph_->updateGraph(current_positions);

        // Calculate Frobenius norm squared: ||L_current - L_desired||²_F
        double similarity_cost;
        if (!swarm_graph_->calcFNorm2(similarity_cost)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to calculate formation similarity");
            return std::numeric_limits<double>::infinity();
        }

        return similarity_cost;
    }


    // Compute Hungarian assignment
    std::vector<int> computeHungarianAssignment(
        const std::vector<Eigen::Vector3d>& current_positions,
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::string& formation_type,
        const std::string& previous_formation_type)
    {
        int n = num_drones_;

        // For line formations, use order-preserving assignment
        if (formation_type.find("line") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "[HUNGARIAN] Order-preserving for line formation: %s", formation_type.c_str());
            std::vector<int> assignment(n);
            std::iota(assignment.begin(), assignment.end(), 0);
            return assignment;
        }

        // For other formations (square, triangle), use Hungarian algorithm
        // Log removed to reduce terminal output

        // Check if it's the same formation type (e.g., square -> square)
        bool same_formation = (formation_type == previous_formation_type);

        if (same_formation) {
            // Same formation - using relative coordinate matching (log removed)

            // Calculate current center (swarm center)
            Eigen::Vector3d current_center = Eigen::Vector3d::Zero();
            for (const auto& pos : current_positions) {
                current_center += pos;
            }
            current_center /= static_cast<double>(n);

            // Calculate target center
            Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
            for (const auto& pos : target_positions) {
                target_center += pos;
            }
            target_center /= static_cast<double>(n);

            // Calculate relative coordinates
            std::vector<Eigen::Vector3d> current_relative(n);
            std::vector<Eigen::Vector3d> target_relative(n);

            for (int i = 0; i < n; ++i) {
                current_relative[i] = current_positions[i] - current_center;
                target_relative[i] = target_positions[i] - target_center;
            }

            // Create cost matrix based on relative coordinates
            auto cost_matrix = path_manager::HungarianAlgorithm::createCostMatrix(current_relative, target_relative);

            // Solve Hungarian algorithm
            auto assignment = path_manager::HungarianAlgorithm::solve(cost_matrix);

            // Relative coordinate assignments computed - detailed logs removed to reduce terminal output

            return assignment;
        } else {
            // Different formation types: use absolute position matching
            RCLCPP_INFO(this->get_logger(), "[HUNGARIAN] Different formation (%s -> %s), using absolute coordinate matching",
                       previous_formation_type.c_str(), formation_type.c_str());

            // Create cost matrix (simple Euclidean distance)
            auto cost_matrix = path_manager::HungarianAlgorithm::createCostMatrix(current_positions, target_positions);

            // Solve Hungarian algorithm
            auto assignment = path_manager::HungarianAlgorithm::solve(cost_matrix);

            // Log results
            RCLCPP_INFO(this->get_logger(), "[HUNGARIAN] Assignments:");
            for (int i = 0; i < n; ++i) {
                RCLCPP_INFO(this->get_logger(), "  Drone %d -> Target %d", i, assignment[i]);
            }

            return assignment;
        }
    }

    void publishFormationCommand()
    {
        if (command_count_ >= (int)current_scenario_.size()) {
            RCLCPP_INFO(this->get_logger(), "Formation command sequence completed. Stopping distance check timer.");
            if (distance_check_timer_) {
                distance_check_timer_->cancel();
            }
            return;
        }

        // Update previous formation type BEFORE publishing new command
        if (command_count_ > 0) {
            previous_formation_type_ = current_scenario_[command_count_ - 1].formation_type;
        }

        path_manager::msg::FormationCommand msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        // Get current command from loaded scenario
        const auto& cmd = current_scenario_[command_count_];

        // Mission sequencing for robustness
        msg.sequence = mission_sequence_;
        msg.current_mission_id = "mission_" + std::to_string(command_count_);

        // Set next mission ID
        if (command_count_ + 1 < (int)current_scenario_.size()) {
            msg.next_mission_id = "mission_" + std::to_string(command_count_ + 1);
            msg.is_final = false;
        } else {
            msg.next_mission_id = "MISSION_END";
            msg.is_final = true;
        }

        msg.formation_type = cmd.formation_type;
        msg.formation_scale = cmd.formation_scale;

        // Convert waypoints
        msg.waypoints.resize(cmd.waypoints.size());
        for (size_t i = 0; i < cmd.waypoints.size(); ++i) {
            msg.waypoints[i].x = cmd.waypoints[i][0];
            msg.waypoints[i].y = cmd.waypoints[i][1];
            msg.waypoints[i].z = cmd.waypoints[i][2];
        }

        // OLD HARDCODED VERSION (kept for reference, can be deleted):
        /*
        switch (command_count_) {
            case 0:
                // Formation center will be the last waypoint: (7.87, -68.99, 0.0)
                msg.formation_type = "line_first";  // Use "line_first" if formation drifts on curves
                msg.formation_scale = 1.0;

                msg.waypoints.resize(5);
                msg.waypoints[0].x = -8.75; msg.waypoints[0].y = -57.0; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = -8.53; msg.waypoints[1].y = -63.3; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = -3.92; msg.waypoints[2].y = -66.04; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 2.11; msg.waypoints[3].y = -68.35; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 7.87; msg.waypoints[4].y = -68.99; msg.waypoints[4].z = 0.0;
                // msg.waypoints[5].x = 16.81; msg.waypoints[5].y = -69.98; msg.waypoints[5].z = 0.0;
                break;

            case 1:
                // Formation center will be the last waypoint: (32.47298, -93.42377, 0.0)
                msg.formation_type = "square";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(5);
                msg.waypoints[0].x = 22.88972; msg.waypoints[0].y = -71.87321; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 30.23262; msg.waypoints[1].y = -74.74562; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 32.45039; msg.waypoints[2].y = -80.23940; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 32.90863; msg.waypoints[3].y = -87.44061; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 32.47298; msg.waypoints[4].y = -93.42377; msg.waypoints[4].z = 0.0;
                break;

            case 2:
                // Stage 1: triangle -> square formation change + initial path
                // Formation center will be the last waypoint: (27.01373, -122.2, 0.0)
                msg.formation_type = "triangle";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(3);
                msg.waypoints[0].x = 32.47298; msg.waypoints[0].y = -93.42377; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 27.85286; msg.waypoints[1].y = -116.42038; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 27.01373; msg.waypoints[2].y = -122.2; msg.waypoints[2].z = 0.0;
                break;

            case 3:
                // Stage 2: square formation maintained, continue long straight path
                // Formation center will be the last waypoint
                msg.formation_type = "square";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(4);
                msg.waypoints[0].x = 27.48601; msg.waypoints[0].y = -127.08222; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 38.37347; msg.waypoints[1].y = -129.20889; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 42.09200; msg.waypoints[2].y = -130.01768; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 100.63176; msg.waypoints[3].y = -138.94522; msg.waypoints[3].z = 0.0;
                break;

            // case 4:
            //     // Combined long path: square -> line_first formation with curved path, circle, and straight segments
            //     msg.formation_center.x = 89.56;
            //     msg.formation_center.y = -26.97;
            //     msg.formation_center.z = 0.0;
            //     msg.formation_type = "line_first";
            //     msg.formation_scale = 2.0;

            //     msg.waypoints.resize(17);
            //     // Initial curved path
            //     msg.waypoints[0].x = 94.61884; msg.waypoints[0].y = -137.36130; msg.waypoints[0].z = 0.0;
            //     msg.waypoints[1].x = 106.19094; msg.waypoints[1].y = -138.85164; msg.waypoints[1].z = 0.0;
            //     msg.waypoints[2].x = 109.88909; msg.waypoints[2].y = -136.74628; msg.waypoints[2].z = 0.0;
            //     msg.waypoints[3].x = 112.11637; msg.waypoints[3].y = -133.03310; msg.waypoints[3].z = 0.0;
            //     msg.waypoints[4].x = 114.09413; msg.waypoints[4].y = -122.49115; msg.waypoints[4].z = 0.0;
            //     // First half of circle path
            //     msg.waypoints[5].x = 116.19815; msg.waypoints[5].y = -106.08804; msg.waypoints[5].z = 0.0;
            //     msg.waypoints[6].x = 121.31105; msg.waypoints[6].y = -104.47885; msg.waypoints[6].z = 0.0;
            //     msg.waypoints[7].x = 129.03363; msg.waypoints[7].y = -101.68507; msg.waypoints[7].z = 0.0;
            //     msg.waypoints[8].x = 133.40306; msg.waypoints[8].y = -96.61195; msg.waypoints[8].z = 0.0;
            //     msg.waypoints[9].x = 137.13039; msg.waypoints[9].y = -88.31696; msg.waypoints[9].z = 0.0;
            //     // Second half of circle path
            //     msg.waypoints[10].x = 134.43517; msg.waypoints[10].y = -77.65039; msg.waypoints[10].z = 0.0;
            //     msg.waypoints[11].x = 128.59427; msg.waypoints[11].y = -72.46378; msg.waypoints[11].z = 0.0;
            //     msg.waypoints[12].x = 121.21349; msg.waypoints[12].y = -68.27953; msg.waypoints[12].z = 0.0;
            //     msg.waypoints[13].x = 123.41044; msg.waypoints[13].y = -52.36917; msg.waypoints[13].z = 0.0;
            //     msg.waypoints[14].x = 124.92587; msg.waypoints[14].y = -37.03749; msg.waypoints[14].z = 0.0;
            //     msg.waypoints[15].x = 122.65699; msg.waypoints[15].y = -32.65216; msg.waypoints[15].z = 0.0;
            //     // Long straight path with obstacles
            //     msg.waypoints[16].x = 89.56; msg.waypoints[16].y = -26.97; msg.waypoints[16].z = 0.0;
            //     break;
            // case 5:
            //     // Middle: line_first -> triangle formation change
            //     msg.formation_center.x = 46.5;
            //     msg.formation_center.y = -20.8;
            //     msg.formation_center.z = 0.0;
            //     msg.formation_type = "triangle";
            //     msg.formation_scale = 3.0;

            //     msg.waypoints.resize(1);
            //     msg.waypoints[0].x = 46.5; msg.waypoints[0].y = -20.8; msg.waypoints[0].z = 0.0;  // Midpoint
            //     break;
            // case 6:
            //     // Final: triangle -> square formation change (earlier transition)
            //     msg.formation_center.x = 20.0;
            //     msg.formation_center.y = -17.0;
            //     msg.formation_center.z = 0.0;
            //     msg.formation_type = "square";
            //     msg.formation_scale = 2.0;

            //     msg.waypoints.resize(4);
            //     msg.waypoints[0].x = 20.0; msg.waypoints[0].y = -17.0; msg.waypoints[0].z = 0.0;
            //     msg.waypoints[1].x = 10.0; msg.waypoints[1].y = -15.5; msg.waypoints[1].z = 0.0;
            //     msg.waypoints[2].x = 3.0; msg.waypoints[2].y = -14.0; msg.waypoints[2].z = 0.0;
            //     msg.waypoints[3].x = 0.04395; msg.waypoints[3].y = 2.61689; msg.waypoints[3].z = 0.0;
            //     break;
        }
        */
        // ========== END OLD HARDCODED VERSION ==========

        // ========== CENTRALIZED HUNGARIAN ALGORITHM ==========
        // Compute target assignments to prevent collision
        //
        // Get current drone positions
        auto current_positions = getCurrentDronePositions();

        // Calculate formation center (last waypoint)
        Eigen::Vector3d formation_center;
        if (!msg.waypoints.empty()) {
            const auto &last_wp = msg.waypoints.back();
            formation_center = Eigen::Vector3d(last_wp.x, last_wp.y, last_wp.z);
            current_formation_center_ = formation_center;
        } else {
            RCLCPP_WARN(this->get_logger(), "No waypoints in FormationCommand, using origin");
            formation_center = Eigen::Vector3d(0, 0, 0);
            current_formation_center_ = formation_center;
        }

        // Generate formation pattern using shared utility
        auto formation_pattern = path_manager::FormationUtils::generateFormationPattern(
            msg.formation_type, num_drones_, msg.formation_scale, formation_z_spacing_);

        // Calculate target positions (formation center + pattern)
        std::vector<Eigen::Vector3d> target_positions(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            target_positions[i] = formation_center + formation_pattern[i];
        }

        // Compute Hungarian assignment
        auto target_assignments = computeHungarianAssignment(
            current_positions, target_positions, msg.formation_type, previous_formation_type_);

        // Add assignments to message
        msg.target_assignments.resize(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            msg.target_assignments[i] = target_assignments[i];
        }

        // Target assignments added - log removed to reduce terminal output

        // ========== END HUNGARIAN ALGORITHM ==========

        formation_cmd_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published formation command #%d (seq: %d): current=%s, next=%s, final=%s at (%.1f, %.1f, %.1f) scale %.1f",
            command_count_ + 1,
            msg.sequence,
            msg.current_mission_id.c_str(),
            msg.next_mission_id.c_str(),
            msg.is_final ? "true" : "false",
            formation_center.x(),
            formation_center.y(),
            formation_center.z(),
            msg.formation_scale
        );

        // Store current message for periodic republishing
        last_published_msg_ = msg;
        last_published_command_count_ = command_count_;

        command_count_++;
        mission_sequence_++;  // Increment sequence for next NEW command
    }

    // Periodic republish for robustness - FSMs can recover if they miss a message
    void periodicRepublish() {
        if (last_published_command_count_ < 0) {
            return;  // No command published yet
        }

        // Republish the last command with updated timestamp
        last_published_msg_.header.stamp = this->now();
        formation_cmd_pub_->publish(last_published_msg_);

        // Log only occasionally to avoid spam
        static int republish_count = 0;
        if (++republish_count % 10 == 0) {
            RCLCPP_DEBUG(this->get_logger(),
                "Periodic republish (seq: %d, current: %s, next: %s)",
                last_published_msg_.sequence,
                last_published_msg_.current_mission_id.c_str(),
                last_published_msg_.next_mission_id.c_str());
        }
    }

    rclcpp::Publisher<path_manager::msg::FormationCommand>::SharedPtr formation_cmd_pub_;
    std::vector<rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr> trajectory_subs_;
    rclcpp::TimerBase::SharedPtr initial_timer_;
    rclcpp::TimerBase::SharedPtr distance_check_timer_;
    rclcpp::TimerBase::SharedPtr periodic_pub_timer_;  // For periodic republishing

    int command_count_;
    int num_drones_;
    double distance_threshold_;
    double formation_similarity_threshold_;
    double formation_z_spacing_;
    Eigen::Vector3d current_formation_center_;
    bool have_initial_command_;
    std::vector<TrajectoryData> drone_trajectories_;
    std::vector<FormationCommand> current_scenario_;  // Loaded scenario from YAML
    std::string previous_formation_type_;  // Track previous formation type for relative matching

    // For robustness: mission sequencing and periodic republishing
    int mission_sequence_;  // Incrementing sequence number for duplicate detection
    int last_published_command_count_;  // Track which command was last published
    path_manager::msg::FormationCommand last_published_msg_;  // Store last message for republishing

    // SwarmGraph for Laplacian-based formation similarity
    SwarmGraph::Ptr swarm_graph_;

    // Load mission commands from YAML
    void loadMissionFromYAML() {
        // Get scenario name from parameter (passed by launch file)
        this->declare_parameter("scenario", "default");
        std::string scenario = this->get_parameter("scenario").as_string();

        // Build path to scenario YAML file
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("path_manager");
        std::string scenario_file = pkg_share_dir + "/config/scenario_" + scenario + ".yaml";

        try {
            YAML::Node yaml = YAML::LoadFile(scenario_file);

            // Navigate to mission.commands
            if (!yaml["/**"]["ros__parameters"]["mission"]["commands"]) {
                RCLCPP_ERROR(this->get_logger(), "No mission.commands found in %s", scenario_file.c_str());
                return;
            }

            YAML::Node commands = yaml["/**"]["ros__parameters"]["mission"]["commands"];

            // Parse each command
            for (size_t i = 0; i < commands.size(); i++) {
                YAML::Node cmd_node = commands[i];

                FormationCommand cmd;
                cmd.formation_type = cmd_node["formation_type"].as<std::string>();
                cmd.formation_scale = cmd_node["formation_scale"].as<double>();

                // Read distance_threshold from YAML, fallback to global parameter if not specified
                if (cmd_node["distance_threshold"]) {
                    cmd.distance_threshold = cmd_node["distance_threshold"].as<double>();
                } else {
                    cmd.distance_threshold = distance_threshold_;
                }

                // Read formation_similarity_threshold from YAML, fallback to global parameter if not specified
                if (cmd_node["formation_similarity_threshold"]) {
                    cmd.formation_similarity_threshold = cmd_node["formation_similarity_threshold"].as<double>();
                } else {
                    cmd.formation_similarity_threshold = formation_similarity_threshold_;
                }

                // Parse waypoints
                YAML::Node waypoints_node = cmd_node["waypoints"];
                for (size_t j = 0; j < waypoints_node.size(); j++) {
                    std::vector<double> waypoint;
                    for (size_t k = 0; k < waypoints_node[j].size(); k++) {
                        waypoint.push_back(waypoints_node[j][k].as<double>());
                    }
                    cmd.waypoints.push_back(waypoint);
                }

                current_scenario_.push_back(cmd);
            }

            RCLCPP_INFO(this->get_logger(), "Loaded %zu formation commands from scenario: %s",
                       current_scenario_.size(), scenario.c_str());

            // Print loaded commands
            for (size_t i = 0; i < current_scenario_.size(); i++) {
                const auto& cmd = current_scenario_[i];
                RCLCPP_INFO(this->get_logger(), "  Command %zu: %s (scale: %.1f, dist_th: %.1f, sim_th: %.1f, %zu waypoints)",
                           i, cmd.formation_type.c_str(), cmd.formation_scale, cmd.distance_threshold,
                           cmd.formation_similarity_threshold, cmd.waypoints.size());
            }

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load scenario YAML: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "File: %s", scenario_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading mission: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FormationCommander>();
    RCLCPP_INFO(node->get_logger(), "FormationCommander node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
