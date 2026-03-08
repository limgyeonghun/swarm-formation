#include "path_manager/replan_fsm.h"
#include <cmath>
#include <sys/resource.h>
#include <numeric>
using namespace std::chrono_literals;

namespace path_manager {

ReplanFSM::ReplanFSM(rclcpp::Node::SharedPtr node)
    : node_(node),
      exec_state_(FSM_EXEC_STATE::INIT),
      continously_called_times_(0),
      have_position_(false),
      have_target_(false),
      have_new_target_(false),
      have_local_traj_(false),
      have_recv_pre_agent_(false),
      flag_replan_astar_(false),
      drone_id_(0),
      replan_thresh_(-1.0),
      no_replan_thresh_(-1.0),
      replan_trajectory_time_(-1.0),
      last_start_time_(0.0),
      n_seconds_ahead_(0.0),
      rviz_simulation_ (false),
      flag_escape_emergency_(false),
      num_drones_(4),
      current_formation_type_("square"),
      current_formation_scale_(2.0),
      has_formation_command_(false),
      last_received_sequence_(-1),
      current_mission_id_(""),
      next_mission_id_(""),
      is_final_mission_(false),
      need_formation_command_sub_(true)
    {
        log_manager_ = std::make_unique<swarm_formation::LogManager>(
            node->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);

    node_->declare_parameter("enable_debug_logs", false);
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);

    node_->declare_parameter("enable_lbfgs_detail_logs", false);
    // Note: enable_lbfgs_detail_logs will be read by PolyTrajOptimizer::setParam()

    node_->declare_parameter("drone_id", 0);
    node_->get_parameter("drone_id", drone_id_);

    node_->declare_parameter("mavlink_id", 1);
    node_->get_parameter("mavlink_id", mavlink_id_);
    FSM_LOG_INFO("Starting ReplanFSM for drone_id: %d (internal), mavlink_id: %d (PX4)", drone_id_, mavlink_id_);

    // Formation manager parameters
    node_->declare_parameter("num_drones", 4);
    node_->declare_parameter("formation_type", "square");
    node_->declare_parameter("formation_scale", 2.0);
    node_->declare_parameter("formation_center_x", 80.0);
    node_->declare_parameter("formation_center_y", -1.5);
    node_->declare_parameter("formation_center_z", 0.0);
    
    node_->get_parameter("num_drones", num_drones_);
    node_->get_parameter("formation_type", current_formation_type_);
    node_->get_parameter("formation_scale", current_formation_scale_);
    
    double center_x, center_y, center_z;
    node_->get_parameter("formation_center_x", center_x);
    node_->get_parameter("formation_center_y", center_y);
    node_->get_parameter("formation_center_z", center_z);
    current_formation_center_ = Eigen::Vector3d(center_x, center_y, center_z);

    node_->declare_parameter("rviz_simulation", false);
    node_->get_parameter("rviz_simulation", rviz_simulation_);
    FSM_LOG_INFO("rviz_simulation: %s", rviz_simulation_ ? "true" : "false");

    node_->declare_parameter("enable_waypoint_markers", true);
    node_->get_parameter("enable_waypoint_markers", enable_waypoint_markers_);
    FSM_LOG_INFO("enable_waypoint_markers: %s", enable_waypoint_markers_ ? "true" : "false");

    node_->declare_parameter("enable_global_trajectory_pub", true);
    node_->get_parameter("enable_global_trajectory_pub", enable_global_trajectory_pub_);
    FSM_LOG_INFO("enable_global_trajectory_pub: %s", enable_global_trajectory_pub_ ? "true" : "false");

    // 3D formation parameters
    node_->declare_parameter("formation_z_spacing", 2.0);
    node_->get_parameter("formation_z_spacing", formation_z_spacing_);
    FSM_LOG_INFO("3D formation z_spacing: %.2f", formation_z_spacing_);

    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/replan_trajectory_time", -1.0);
    node_->declare_parameter("fsm/n_seconds_ahead", -1.0);
    node_->declare_parameter("fsm/hungarian_distance_weight", 1.0);
    node_->declare_parameter("fsm/hungarian_crossing_penalty", 50.0);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/replan_trajectory_time", replan_trajectory_time_);
    node_->get_parameter("fsm/n_seconds_ahead", n_seconds_ahead_);
    node_->get_parameter("fsm/hungarian_distance_weight", hungarian_distance_weight_);
    node_->get_parameter("fsm/hungarian_crossing_penalty", hungarian_crossing_penalty_);
    FSM_LOG_INFO("Hungarian assignment weights - distance: %.1f, crossing penalty: %.1f",
                 hungarian_distance_weight_, hungarian_crossing_penalty_);

    // Read nonholonomic weight from config (will be used for formation changes)
    if (!node_->has_parameter("optimization/weight_nonholonomic")) {
        node_->declare_parameter("optimization/weight_nonholonomic", 10000.0);
    }
    node_->get_parameter("optimization/weight_nonholonomic", weight_nonholonomic_);
    pending_weight_nonholonomic_ = weight_nonholonomic_;  // Default: use config value
    FSM_LOG_INFO("Nonholonomic weight from config: %.1f (will be 0 for line formations, this value for others)",
                 weight_nonholonomic_);

    // Start position will be received from TrajectoryCommand message
    // Initialize with zero until we receive the command
    RCLCPP_INFO(node_->get_logger(), "ReplanFSM parameters:");
    log_manager_->infof("ReplanFSM parameters:");
    RCLCPP_INFO(node_->get_logger(), "  Start position will be set from TrajectoryCommand");
    log_manager_->infof("  Start position will be set from TrajectoryCommand");
    RCLCPP_INFO(node_->get_logger(), "  Waiting for trajectory command...");
    log_manager_->infof("  Waiting for trajectory command...");

    offset_pt_ = Eigen::Vector3d::Zero();
    start_pt_ = offset_pt_;
    current_pos_ = offset_pt_;  // Initialize current_pos_ to zero, will be updated from TrajectoryCommand
    current_vel_ = Eigen::Vector3d::Zero();  // Initialize velocity to zero
    end_pt_ = Eigen::Vector3d::Zero();  // Initialize end_pt_ to avoid uninitialized access
    prev_end_pt_ = Eigen::Vector3d::Zero();  // Initialize previous endpoint
    prev_formation_offset_ = Eigen::Vector3d::Zero();  // Initialize previous formation offset
    have_target_ = false;  // Wait for trajectory command
    start_position_received_ = false;  // Flag to track if we received start position
    
    RCLCPP_INFO(node_->get_logger(), "Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));
    log_manager_->infof("Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));

    // Initialize PathManager in constructor to avoid nullptr access
    path_manager_ = std::make_shared<PathManager>(node_);

    // SwarmGraph removed - formation management now handled by formation_manager package

    // Initialize swarm_positions_ map for all drones
    for (int i = 0; i < num_drones_; ++i) {
        swarm_positions_[i] = Eigen::Vector3d::Zero();  // Will be updated from broadcast
    }
    FSM_LOG_INFO("Initialized swarm_positions_ for %d drones", num_drones_);

    RCLCPP_INFO(node_->get_logger(), "PathManager initialized, waiting for formation command from formation_commander");
    log_manager_->infof("PathManager initialized, waiting for formation command from formation_commander");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create callback groups with dedicated separation to prevent timer stalls:
    // - timer_callback_group: FSM timer only (MutuallyExclusive, runs independently)
    // - subscription_callback_group: Formation/broadcast callbacks (MutuallyExclusive)
    // - position_callback_group: Position updates (MutuallyExclusive, separate to avoid blocking)
    // With MultiThreadedExecutor, these groups can run in parallel threads
    timer_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    position_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    FSM_LOG_INFO("Callback groups created: timer, subscription, position (all MutuallyExclusive)");

    std::string odom_topic = "/vehicle" + std::to_string(drone_id_+1) + "/target_position";
    std::string topic_prefix = "/V" + std::to_string(drone_id_+1);

    optimized_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(topic_prefix + "/planning/trajectory", sensor_qos);
    global_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(topic_prefix + "/planning/global", sensor_qos);
    broadcast_traj_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(topic_prefix + "/planning/broadcast_traj_send", sensor_qos);

    // Only FSM1 (drone_id == 0) publishes verified trajectories to formation_commander
    if (drone_id_ == 0) {
        verified_traj_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>("/for_commander/trajectories", sensor_qos);
        FSM_LOG_INFO("FSM1: Created verified trajectory publisher for formation_commander");
    }

    // Position callback uses separate callback group to ensure it always runs
    // even when main callback group is blocked by long trajectory planning
    rclcpp::SubscriptionOptions position_options;
    position_options.callback_group = position_callback_group_;

    if (rviz_simulation_)
    {
        // Internal agent topic for simulation
        std::string target_position_topic = "/agent" + std::to_string(drone_id_) + "/target_position";
        target_position_sub_ = node_->create_subscription<path_manager::msg::PositionCommand>(
            target_position_topic, sensor_qos,
            std::bind(&ReplanFSM::targetPositionCallback, this, std::placeholders::_1),
            position_options);
        have_position_ = true;  // We have initial position from parameters
    }
    else
    {
#ifdef HAVE_PX4_MSGS
        // External MAVLink topic for real PX4
        std::string px4_position_topic = "/vehicle" + std::to_string(mavlink_id_) + "/fmu/out/vehicle_local_position";
        px4_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            px4_position_topic, sensor_qos,
            std::bind(&ReplanFSM::PX4positionCallback, this, std::placeholders::_1),
            position_options);
#else
        FSM_LOG_WARN("PX4 support disabled - cannot subscribe to vehicle_local_position. Use rviz_simulation mode.");
#endif
    }

    rclcpp::SubscriptionOptions broadcast_options;
    broadcast_options.callback_group = subscription_callback_group_;
    broadcast_traj_sub_ = node_->create_subscription<path_manager::msg::PolyTraj>(
        topic_prefix + "/j_fi/broadcast_traj_recv", sensor_qos,
        std::bind(&ReplanFSM::recvBroadcastPolyTrajCallback, this, std::placeholders::_1),
        broadcast_options);

    rclcpp::SubscriptionOptions formation_target_options;
    formation_target_options.callback_group = subscription_callback_group_;
    formation_target_sub_ = node_->create_subscription<path_manager::msg::FormationTarget>(
        topic_prefix + "/formation_target", sensor_qos,
        std::bind(&ReplanFSM::formationTargetCallback, this, std::placeholders::_1),
        formation_target_options);

    rclcpp::SubscriptionOptions trajectory_cmd_options;
    trajectory_cmd_options.callback_group = subscription_callback_group_;
    trajectory_cmd_sub_ = node_->create_subscription<formation_msgs::msg::TrajectoryCommand>(
        topic_prefix + "/trajectory_command", sensor_qos,
        std::bind(&ReplanFSM::trajectoryCommandCallback, this, std::placeholders::_1),
        trajectory_cmd_options);

    formation_target_pub_ = node_->create_publisher<path_manager::msg::FormationTarget>(
        topic_prefix + "/formation_target", sensor_qos);

    waypoint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "waypoint_markers", 10);

    timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this), timer_callback_group_);
    FSM_LOG_INFO("FSM timer created with dedicated callback group (10ms period)");
}

void ReplanFSM::init()
{
    path_manager_->initOptimizer();
    path_manager_->deliverTrajToOptimizer();

    // Only plan global trajectory if we have a valid target
    if (!have_target_) {
        RCLCPP_WARN(node_->get_logger(), "No target set yet, skipping global trajectory planning");
        log_manager_->warnf("No target set yet, skipping global trajectory planning");
        return;
    }

    try {
        Eigen::MatrixXd iniState = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd finState = Eigen::MatrixXd::Zero(3, 3);
        iniState.col(0) = start_pt_;
        finState.col(0) = end_pt_;

        bool success = path_manager_->planGlobalTraj(start_pt_, iniState.col(1), iniState.col(2),
                                                     {end_pt_}, finState.col(1), finState.col(2));
        if (success)
        {
            FSM_LOG_INFO("Success to generate global trajectory!!!");
            // end_vel_.setZero();
            have_target_ = true;
            have_new_target_ = true;

            if (exec_state_ == WAIT_POSITION)
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            else if (exec_state_ == EXEC_TRAJ)
                changeFSMExecState(REPLAN_TRAJ, "TRIG");
                
            if (enable_global_trajectory_pub_) {
                path_manager::msg::PolyTraj msg;
                globalTraj2ROSMsg(msg);
                global_path_pub_->publish(msg);
                FSM_LOG_INFO("Published global trajectory successfully!");
            }
        }
        else
        {
            FSM_LOG_ERROR("Failed to generate global trajectory!!!");
        }
    } catch (const std::exception& e) {
        FSM_LOG_ERROR("Exception during global trajectory planning: %s", e.what());
    }
}

void ReplanFSM::computeAndPublishPaths() {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        fsm_num = 0;
    }

    switch (exec_state_) {
        case INIT: {
            if (!have_position_) {
                return;
            }
            changeFSMExecState(WAIT_POSITION, "FSM");
            break;
        }

        case WAIT_POSITION: {
            if (!have_target_) {
                return;
            }
            changeFSMExecState(SEQUENTIAL_START, "FSM");
            break;
        }

        case SEQUENTIAL_START: {
            if (drone_id_ <= 0 || (drone_id_ >= 1 && have_recv_pre_agent_)) 
            {
                bool success = false;
                try {
                    success = planFromGlobalTraj(1);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during trajectory planning in SEQUENTIAL_START: %s", e.what());
                    log_manager_->errorf("Exception during trajectory planning in SEQUENTIAL_START: %s", e.what());
                    success = false;
                }
                
                static int sequential_start_failures = 0;  // Track consecutive failures

                if (success)
                {
                    sequential_start_failures = 0;  // Reset on success
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    log_manager_->errorf("MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    // Instead of going back to SEQUENTIAL_START immediately, wait a bit
                    sequential_start_failures++;
                    if (sequential_start_failures > 10) {
                        RCLCPP_ERROR(node_->get_logger(), "Too many failures in SEQUENTIAL_START, going to EMERGENCY_STOP");
                        log_manager_->errorf("Too many failures in SEQUENTIAL_START, going to EMERGENCY_STOP");
                        changeFSMExecState(EMERGENCY_STOP, "FSM");
                        sequential_start_failures = 0;
                    } else {
                        changeFSMExecState(WAIT_POSITION, "FSM");
                    }
                }
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            bool success = planFromGlobalTraj(1);
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } else {
                have_target_ = false;
                changeFSMExecState(WAIT_POSITION, "FSM");
            }
            break;
        }

        case REPLAN_TRAJ:
        {
            // FSM_LOG_INFO("[DEBUG REPLAN] Starting REPLAN_TRAJ at time %.3f",
            //              rclcpp::Clock(RCL_ROS_TIME).now().seconds());
            auto replan_start = std::chrono::high_resolution_clock::now();

            bool success;
            if (flag_replan_astar_)
                success = planFromLocalTraj(true, false);
            else
                success = planFromLocalTraj(false, true);

            auto replan_end = std::chrono::high_resolution_clock::now();
            auto replan_duration = std::chrono::duration_cast<std::chrono::milliseconds>(replan_end - replan_start).count();
            // FSM_LOG_INFO("[DEBUG REPLAN] planFromLocalTraj took %ld ms, success=%d",
            //              replan_duration, success);

            if (success)
            {
                flag_replan_astar_ = false;
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
            else
            {
                flag_replan_astar_ = true;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case EXEC_TRAJ:
        {
            if (!path_manager_) {
                RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
                log_manager_->errorf("PathManager is not initialized!");
                changeFSMExecState(EMERGENCY_STOP, "FSM");
                break;
            }
            auto local_traj = &path_manager_->traj_.local_traj;
            double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - local_traj->start_time;
            t_cur = std::min(local_traj->duration, t_cur);

	        //RCLCPP_INFO(node_->get_logger(), "t_cur: %.2f, duration: %.2f", t_cur, local_traj->duration);
            Eigen::Vector3d pos = local_traj->traj.getPos(t_cur);

            if ((local_target_pt_ - end_pt_).norm() < 0.1)
            {
                if (t_cur > local_traj->duration - 0.2)
                {
                    have_target_ = false;
                    have_local_traj_ = false;
                    changeFSMExecState(WAIT_POSITION, "FSM");
                    RCLCPP_INFO(node_->get_logger(), "[drone %d reached goal]", drone_id_);
                    log_manager_->infof("[drone %d reached goal]", drone_id_);
                    return;
                }
                else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
                {
                    log_manager_->errorf("No Replan Thresh");
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
                }
            }
            else if (t_cur > replan_thresh_)
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case EMERGENCY_STOP: {
            if (flag_escape_emergency_) {
                // Avoiding repeated calls
                callEmergencyStop(current_pos_);
            } else {
                // Check if drone has stopped (velocity near zero)
                // If stopped, try to generate new trajectory
                if (current_vel_.norm() < 0.1) {
                    RCLCPP_WARN(node_->get_logger(), "Drone stopped, attempting to recover from emergency stop");
                    log_manager_->warnf("Drone stopped, attempting to recover from emergency stop");
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
            }
            flag_escape_emergency_ = false;
            break;
        }
    }
}

void ReplanFSM::targetPositionCallback(const path_manager::msg::PositionCommand::SharedPtr msg) {
    Eigen::Vector3d new_pos(msg->position.x, msg->position.y, msg->position.z);

    current_pos_ = new_pos;
    swarm_positions_[drone_id_] = new_pos;
}

#ifdef HAVE_PX4_MSGS
void ReplanFSM::PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    Eigen::Vector3d new_pos;
    new_pos(0) = msg->x + offset_pt_(0);
    new_pos(1) = msg->y + offset_pt_(1);
    new_pos(2) = offset_pt_(2);

    // Sanity check: detect position jumps (likely from PX4 sensor glitches)
    if (have_position_) {
        double position_jump = (new_pos - current_pos_).norm();
        const double MAX_POSITION_JUMP = 50.0;  // 50m threshold

        if (position_jump > MAX_POSITION_JUMP) {
            FSM_LOG_WARN("PX4 position jump detected! Distance: %.2fm, rejecting update. "
                        "Old: (%.2f, %.2f, %.2f), New: (%.2f, %.2f, %.2f)",
                        position_jump,
                        current_pos_(0), current_pos_(1), current_pos_(2),
                        new_pos(0), new_pos(1), new_pos(2));
            return;  // Reject this position update
        }
    }

    current_pos_ = new_pos;
    swarm_positions_[drone_id_] = new_pos;
    have_position_ = true;
}
#endif

void ReplanFSM::recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    auto callback_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_DEBUG("[CALLBACK START] recvBroadcastPolyTrajCallback");

    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    if (msg->drone_id < 0 || msg->drone_id >= num_drones_) {
        RCLCPP_ERROR(node_->get_logger(),
                    "Invalid drone_id: %d (valid range: 0-%d)",
                    msg->drone_id, num_drones_-1);
        log_manager_->errorf("Invalid drone_id: %d (valid range: 0-%d)",
                            msg->drone_id, num_drones_-1);
        return;
    }
    if (msg->order != 5) {
        RCLCPP_ERROR(node_->get_logger(), "Only support trajectory order equals 5 now!");
        log_manager_->errorf("Only support trajectory order equals 5 now!");
        return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
        RCLCPP_ERROR(node_->get_logger(), "WRONG trajectory parameters.");
        log_manager_->errorf("WRONG trajectory parameters.");
        return;
    }
    rclcpp::Time msg_time(msg->start_time);
    double time_diff = (rclcpp::Clock(RCL_ROS_TIME).now() - msg_time).seconds();
    if (std::abs(time_diff) > 0.25) {
        RCLCPP_WARN(node_->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs",
                   msg->drone_id, time_diff);
        log_manager_->warnf("Time stamp diff: Local - Remote Agent %d = %fs",
                   msg->drone_id, time_diff);
        return;
    }

    const size_t recv_id = static_cast<size_t>(msg->drone_id);
    if (static_cast<int>(recv_id) == drone_id_) {
        return;
    }

    /* Fill up the buffer */
    if (path_manager_->traj_.swarm_traj.size() <= recv_id) {
        for (size_t i = path_manager_->traj_.swarm_traj.size(); i <= recv_id; i++) {
            LocalTrajData blank;
            blank.drone_id = -1;
            path_manager_->traj_.swarm_traj.push_back(blank);
        }
    }

    /* Store data */
    path_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    path_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    path_manager_->traj_.swarm_traj[recv_id].start_time = msg_time.seconds();

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

    poly_traj::Trajectory trajectory(dura, cMats);
    path_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
    path_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    path_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    swarm_positions_[recv_id] = trajectory.getPos(0.0);

    // Publish verified trajectory to formation_commander (other drones' trajectories, FSM1 only)
    if (verified_traj_pub_) {
        verified_traj_pub_->publish(*msg);
    }

    if (path_manager_->checkCollision(recv_id)) {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_) {
        if (static_cast<int>(path_manager_->traj_.swarm_traj.size()) >= drone_id_) {
            for (int i = 0; i < drone_id_; ++i) {
                if (path_manager_->traj_.swarm_traj[i].drone_id != i) {
                    break;
                }
                have_recv_pre_agent_ = true;
            }
        }
    }

    auto callback_end = std::chrono::high_resolution_clock::now();
    auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start).count();
    FSM_LOG_DEBUG("[CALLBACK END] recvBroadcastPolyTrajCallback (took %ld us)", callback_duration);
}

void ReplanFSM::polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg)
{
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    auto data = &path_manager_->traj_.local_traj;

    msg.drone_id = drone_id_;
    msg.traj_id = data->traj_id;
    msg.order = 5;

    const double s = data->start_time;
    msg.start_time.sec     = static_cast<int32_t>(std::floor(s));
    msg.start_time.nanosec = static_cast<uint32_t>(std::llround((s - msg.start_time.sec) * 1e9));

    msg.is_final_mission = is_final_mission_;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
}

void ReplanFSM::globalTraj2ROSMsg(path_manager::msg::PolyTraj &msg) 
{
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    msg.drone_id = drone_id_;

    auto data = &path_manager_->traj_.global_traj;

    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    msg.start_time.sec = now.seconds();
    msg.start_time.nanosec = now.nanoseconds() % 1000000000;
    msg.order = 5;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
}

bool ReplanFSM::callPathManager(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation) {
    auto replan_start = std::chrono::high_resolution_clock::now();

    log_manager_->infof("[callPathManager] ENTER: start=(%.2f,%.2f,%.2f), end=(%.2f,%.2f,%.2f), use_formation=%d, have_local_traj=%d",
                 start_pt_(0), start_pt_(1), start_pt_(2),
                 end_pt_(0), end_pt_(1), end_pt_(2),
                 use_formation, have_local_traj_);

    path_manager_->getLocalTarget(start_pt_, end_pt_, local_target_pt_, local_target_vel_, t_to_target_);

    log_manager_->infof("[callPathManager] After getLocalTarget: local_target=(%.2f,%.2f,%.2f), local_vel=(%.2f,%.2f,%.2f)",
                 local_target_pt_(0), local_target_pt_(1), local_target_pt_(2),
                 local_target_vel_(0), local_target_vel_(1), local_target_vel_(2));

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;
    double desired_start_time;

    if (have_local_traj_ && use_formation) {
        desired_start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds() + replan_trajectory_time_;
        double t_adj = desired_start_time - path_manager_->traj_.local_traj.start_time;
        desired_start_pt = path_manager_->traj_.local_traj.traj.getPos(t_adj);
        desired_start_vel = path_manager_->traj_.local_traj.traj.getVel(t_adj);
        desired_start_acc = path_manager_->traj_.local_traj.traj.getAcc(t_adj);
    } else {
        desired_start_pt = start_pt_;
        desired_start_vel = start_vel_;
        desired_start_acc = start_acc_;
    }

    auto optimize_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_DEBUG("[TIMING] Starting computeAndOptimizePath (use_formation=%d)", use_formation);

    bool plan_success = path_manager_-> computeAndOptimizePath(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, use_formation, have_local_traj_);

    auto optimize_end = std::chrono::high_resolution_clock::now();
    auto optimize_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimize_end - optimize_start).count();
    FSM_LOG_DEBUG("[TIMING] computeAndOptimizePath took %ld ms", optimize_duration);

    have_new_target_ = false;

    if (enable_global_trajectory_pub_) {
        FSM_LOG_DEBUG("Publishing global trajectory...");
        path_manager::msg::PolyTraj msg2;
        globalTraj2ROSMsg(msg2);
        global_path_pub_->publish(msg2);
        FSM_LOG_DEBUG("Global trajectory published");
    }

    if (plan_success) {
        FSM_LOG_DEBUG("Publishing local trajectories...");
        path_manager::msg::PolyTraj msg;
        polyTraj2ROSMsg(msg);

        FSM_LOG_DEBUG("Publishing to planning/trajectory...");
        auto pub_start_1 = std::chrono::high_resolution_clock::now();
        optimized_path_pub_->publish(msg);
        auto pub_end_1 = std::chrono::high_resolution_clock::now();
        auto pub_duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(pub_end_1 - pub_start_1).count();
        FSM_LOG_DEBUG("Published to planning/trajectory (took %ld us)", pub_duration_1);

        FSM_LOG_DEBUG("Publishing to broadcast_traj_send...");
        auto pub_start_2 = std::chrono::high_resolution_clock::now();
        broadcast_traj_pub_->publish(msg);
        if (verified_traj_pub_) {
            verified_traj_pub_->publish(msg);  // Also publish to formation_commander (own trajectory, FSM1 only)
        }
        auto pub_end_2 = std::chrono::high_resolution_clock::now();
        auto pub_duration_2 = std::chrono::duration_cast<std::chrono::microseconds>(pub_end_2 - pub_start_2).count();
        FSM_LOG_DEBUG("Published to broadcast_traj_send (took %ld us)", pub_duration_2);

        // Warn if publishing took too long (potential blocking issue)
        if (pub_duration_2 > 1000) { // > 1ms
            FSM_LOG_WARN("[PUBLISH DELAY] broadcast_traj_send publish took %ld us", pub_duration_2);
        }

        have_local_traj_ = true;
    }

    FSM_LOG_DEBUG("callPathManager returning %s", plan_success ? "SUCCESS" : "FAILURE");
    return plan_success;
}

bool ReplanFSM::planFromGlobalTraj(int trial_times) {
    start_pt_ = current_pos_;
    start_vel_.setZero();
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++) {
        try {
            if (callPathManager(true, false, true)) {
                return true;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in planFromGlobalTraj trial %d: %s", i, e.what());
            log_manager_->errorf("Exception in planFromGlobalTraj trial %d: %s", i, e.what());
        }
    }
    return false;
}

bool ReplanFSM::planFromLocalTraj(bool flag_use_poly_init, bool use_formation) {
    LocalTrajData *info = &path_manager_->traj_.local_traj;
    double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - path_manager_->traj_.local_traj.start_time;

    // Always use trajectory position for consistency and continuity
    // If trajectory expired, clamp to duration to get the hover endpoint
    // This avoids race conditions with current_pos_ (updated by different callback group)
    // and ensures smooth velocity/acceleration continuity
    double t_clamped = std::min(t_cur, info->duration);

    start_pt_ = info->traj.getPos(t_clamped);
    start_vel_ = info->traj.getVel(t_clamped);
    start_acc_ = info->traj.getAcc(t_clamped);

    // Log if trajectory expired (indicates timer was delayed)
    if (t_cur > info->duration) {
        FSM_LOG_WARN("Trajectory expired! t_cur=%.2fs > duration=%.2fs, using trajectory endpoint (clamped)",
                     t_cur, info->duration);
        FSM_LOG_DEBUG("  Trajectory endpoint: (%.2f, %.2f, %.2f), Current pos: (%.2f, %.2f, %.2f), Error: %.2fm",
                     start_pt_(0), start_pt_(1), start_pt_(2),
                     current_pos_(0), current_pos_(1), current_pos_(2),
                     (start_pt_ - current_pos_).norm());
    }

    double t_ahead = std::min(t_cur + n_seconds_ahead_, info->duration);
    Eigen::Vector3d desired_start_pt = info->traj.getPos(std::min(t_cur, info->duration));

    bool success = callPathManager(flag_use_poly_init, false, use_formation);
    return success;
}

void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
    if (new_state == exec_state_) {
        continously_called_times_++;
    } else {
        continously_called_times_ = 1;
    }

    static std::string state_str[7] = {"INIT", "WAIT_POSITION", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    // Throttle frequent state transitions (EXEC_TRAJ <-> REPLAN_TRAJ)
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();

    bool should_log = (exec_state_ != new_state) &&  // State actually changed
                      (new_state == WAIT_POSITION || new_state == SEQUENTIAL_START || new_state == GEN_NEW_TRAJ || elapsed >= 2);

    if (should_log) {
        RCLCPP_INFO(node_->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
        last_log_time = now;
    }

    log_manager_->infof("[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
    exec_state_ = new_state;
}

void ReplanFSM::formationTargetCallback(const path_manager::msg::FormationTarget::SharedPtr msg) {
    auto callback_start = std::chrono::high_resolution_clock::now();
    double ros_time_start = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

    if (msg->drone_id != drone_id_) {
        return;
    }

    if (msg->target_position.z < -0.1) {
        return;
    }

    FSM_LOG_INFO("[DEBUG TARGET] formationTargetCallback STARTED for drone %d at time %.3f!",
                 drone_id_, ros_time_start);
    FSM_LOG_INFO("[DEBUG TARGET] Message header timestamp: %.3f, delay: %.3f ms",
                 msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                 (ros_time_start - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9)) * 1000);

    // Mission progress: move next_mission to current, clear next
    // This signals that we're working on the "next" mission now (which becomes current)
    if (!next_mission_id_.empty() && next_mission_id_ != "MISSION_END") {
        current_mission_id_ = next_mission_id_;
        next_mission_id_.clear();  // Empty means we need new mission data
        FSM_LOG_INFO("Mission progressed: now executing %s, next mission cleared (will request new data)",
                    current_mission_id_.c_str());
    }

    // Initialize optimizer if not already initialized
    if (!path_manager_->isOptimizerInitialized()) {
        auto opt_init_start = std::chrono::high_resolution_clock::now();
        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", drone_id_);
            log_manager_->infof("Initializing optimizer for drone %d...", drone_id_);
            FSM_LOG_INFO("[TIMING] Optimizer initialization started");

            path_manager_->initOptimizer();
            path_manager_->deliverTrajToOptimizer();

            auto opt_init_end = std::chrono::high_resolution_clock::now();
            auto opt_init_duration = std::chrono::duration_cast<std::chrono::milliseconds>(opt_init_end - opt_init_start).count();

            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", drone_id_);
            log_manager_->infof("Optimizer initialized successfully for drone %d", drone_id_);
            FSM_LOG_INFO("[TIMING] Optimizer initialization took %ld ms", opt_init_duration);

            // Apply pending nonholonomic weight (set by formation command)
            path_manager_->setNonholonomicWeight(pending_weight_nonholonomic_);
            FSM_LOG_INFO("Applied pending nonholonomic weight: %.1f", pending_weight_nonholonomic_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            log_manager_->errorf("Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            return;
        }
    }

    if (have_local_traj_) {
        LocalTrajData *info = &path_manager_->traj_.local_traj;
        double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - info->start_time;
        Eigen::Vector3d theoretical_pos = info->traj.getPos(t_cur);
        Eigen::Vector3d theoretical_vel = info->traj.getVel(t_cur);
        Eigen::Vector3d theoretical_acc = info->traj.getAcc(t_cur);

        // Use trajectory position for smooth formation change
        // This prevents jumps in commanded position during formation transitions
        start_pt_ = theoretical_pos;
        start_vel_ = theoretical_vel;
        start_acc_ = theoretical_acc;

        double pos_error = (current_pos_ - theoretical_pos).norm();
        log_manager_->infof("Formation change - using TRAJECTORY position/vel/acc (error from actual: %.2fm)",
                   pos_error);
        log_manager_->infof("  Trajectory pos: (%.2f, %.2f, %.2f), Actual pos: (%.2f, %.2f, %.2f)",
                   start_pt_(0), start_pt_(1), start_pt_(2),
                   current_pos_(0), current_pos_(1), current_pos_(2));
    } else {
        start_pt_ = current_pos_;
        start_vel_ = Eigen::Vector3d::Zero();
        start_acc_ = Eigen::Vector3d::Zero();
        log_manager_->infof("Using current position (no trajectory yet): (%.2f, %.2f, %.2f)",
                   start_pt_(0), start_pt_(1), start_pt_(2));
    }

    bool success = false;

    std::vector<Eigen::Vector3d> waypoints;

    if (!msg->formation_positions.empty()) {
        for (const auto& pos : msg->formation_positions) {
            waypoints.emplace_back(pos.x, pos.y, pos.z);
        }

        log_manager_->infof(
                   "Drone %d: Using %zu waypoints from formation_positions (already offset-applied)",
                   drone_id_, waypoints.size());

        end_pt_ = waypoints.back();
    } else {
        Eigen::Vector3d my_target(
            msg->target_position.x,
            msg->target_position.y,
            msg->target_position.z);

        waypoints = { my_target };
        end_pt_ = my_target;

        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d: Using single target_position as waypoint (already offset-applied)",
                   drone_id_);
        log_manager_->infof(
                   "Drone %d: Using single target_position as waypoint (already offset-applied)",
                   drone_id_);
    }

    log_manager_->infof("start_pt_: %.2f, %.2f, %.2f", start_pt_(0), start_pt_(1), start_pt_(2));
    log_manager_->infof("waypoints size: %zu", waypoints.size());
    for (const auto& wp : waypoints) {
        log_manager_->infof("waypoint: %.2f, %.2f, %.2f", wp(0), wp(1), wp(2));
    }

    if (enable_waypoint_markers_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints_drone_" + std::to_string(drone_id_);
        marker.id = drone_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;  // Sphere size

        if (drone_id_ == 0) {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;  // Red
        } else if (drone_id_ == 1) {
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;  // Blue
        } else if (drone_id_ == 2) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;  // Green
        } else if (drone_id_ == 3) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;  // Yellow
        }
        marker.color.a = 1.0;

        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = wp(0);
            p.y = wp(1);
            p.z = wp(2);
            marker.points.push_back(p);
        }

        waypoint_marker_pub_->publish(marker);
        log_manager_->infof("Drone %d: Published %zu waypoint markers to RViz (frame: %s, ns: %s)",
                    drone_id_, waypoints.size(), marker.header.frame_id.c_str(), marker.ns.c_str());
    }

    // Use formation pattern received from formation_manager via TrajectoryCommand
    auto formation_setup_start = std::chrono::high_resolution_clock::now();
    path_manager_->setFormationInfo(drone_id_, current_formation_type_, current_formation_pattern_);

    auto global_traj_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_INFO("[TIMING] Starting global trajectory planning");
    success = path_manager_->planGlobalTraj(
        start_pt_, start_vel_, start_acc_,
        waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto global_traj_end = std::chrono::high_resolution_clock::now();
    auto global_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(global_traj_end - global_traj_start).count();
    FSM_LOG_INFO("[TIMING] Global trajectory planning took %ld ms", global_traj_duration);

    if (success) {
        // Pass formation pattern to optimizer for formation cost calculation
        FSM_LOG_INFO("Trajectory planning successful for formation type: %s", current_formation_type_.c_str());

        if (!current_formation_pattern_.empty()) {
            log_manager_->infof("Formation pattern for optimizer (relative coordinates):");
            for (size_t i = 0; i < current_formation_pattern_.size(); ++i) {
                log_manager_->infof("  Drone %zu: [%.3f, %.3f, %.3f]",
                                   i, current_formation_pattern_[i].x(),
                                   current_formation_pattern_[i].y(),
                                   current_formation_pattern_[i].z());
            }
            path_manager_->setFormationToOptimizer(current_formation_pattern_, num_drones_);
        } else {
            FSM_LOG_WARN("Formation pattern is empty, optimizer may not apply formation constraints");
            path_manager_->setFormationToOptimizer(current_formation_pattern_, num_drones_);
        }

        // Save current endpoint and offset for next formation transition
        prev_end_pt_ = end_pt_;
        prev_formation_offset_ = Eigen::Vector3d(
            msg->formation_offset.x,
            msg->formation_offset.y,
            msg->formation_offset.z
        );
        log_manager_->infof(
                   "Drone %d: Saved endpoint (%.2f, %.2f, %.2f) and offset (%.2f, %.2f, %.2f) for next transition",
                   drone_id_,
                   prev_end_pt_.x(), prev_end_pt_.y(), prev_end_pt_.z(),
                   prev_formation_offset_.x(), prev_formation_offset_.y(), prev_formation_offset_.z());

        have_target_ = true;
        have_new_target_ = true;

        if (exec_state_ == WAIT_POSITION)
            changeFSMExecState(SEQUENTIAL_START, "formationTargetCallback");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "formationTargetCallback");

        // NOTE: Global trajectory publish removed from formationTargetCallback to prevent blocking
        // The global trajectory will be published in computeAndPublishPaths instead
        // This fixes the race condition that caused 20-60 second freezes during formation transitions

        RCLCPP_INFO(node_->get_logger(), "Successfully generated global trajectory for drone %d", drone_id_);
        log_manager_->infof("Successfully generated global trajectory for drone %d", drone_id_);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory for drone %d!", drone_id_);
        log_manager_->errorf("Unable to generate global trajectory for drone %d!", drone_id_);
    }

    auto callback_end = std::chrono::high_resolution_clock::now();
    auto callback_duration = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end - callback_start).count();
    double ros_time_end = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    FSM_LOG_INFO("[DEBUG TARGET] formationTargetCallback COMPLETED in %ld ms at time %.3f (total elapsed: %.3f ms)",
                 callback_duration, ros_time_end, (ros_time_end - ros_time_start) * 1000);
}
bool ReplanFSM::isMapReady(const Eigen::Vector3d& start_pos) {
    if (!path_manager_) {
        RCLCPP_WARN(node_->get_logger(), "PathManager not initialized yet");
        log_manager_->warnf("PathManager not initialized yet");
        return false;
    }

    bool map_ready = path_manager_->isMapReady(start_pos);
    if (!map_ready) {
        RCLCPP_DEBUG(node_->get_logger(), "Map not ready for position (%f,%f,%f)",
                    start_pos(0), start_pos(1), start_pos(2));
        log_manager_->debugf("Map not ready for position (%f,%f,%f)",
                    start_pos(0), start_pos(1), start_pos(2));
    }

    return map_ready;
}

bool ReplanFSM::callEmergencyStop(const Eigen::Vector3d& stop_pos) {
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return false;
    }

    path_manager_->EmergencyStop(stop_pos);

    path_manager::msg::PolyTraj msg;
    polyTraj2ROSMsg(msg);
    optimized_path_pub_->publish(msg);
    broadcast_traj_pub_->publish(msg);
    if (verified_traj_pub_) {
        verified_traj_pub_->publish(msg);  // Also publish to formation_commander (FSM1 only)
    }

    return true;
}

// trajectoryCommandCallback: receives individual trajectory command from formation_manager
// The target position and waypoints are already calculated by formation_manager
void ReplanFSM::trajectoryCommandCallback(const formation_msgs::msg::TrajectoryCommand::SharedPtr msg) {
    auto callback_start = std::chrono::high_resolution_clock::now();
    FSM_LOG_INFO("[TRAJECTORY CMD] Received trajectory command (seq: %d, drone: %d) at time %.3f",
                 msg->sequence, msg->drone_id, rclcpp::Clock(RCL_ROS_TIME).now().seconds());

    // Check if this command is for this drone
    if (msg->drone_id != drone_id_) {
        FSM_LOG_DEBUG("Ignoring trajectory command for drone %d (I am drone %d)",
                     msg->drone_id, drone_id_);
        return;
    }

    // Check for duplicate messages using sequence number
    if (msg->sequence <= last_received_sequence_) {
        FSM_LOG_DEBUG("Ignoring duplicate/old trajectory command (seq: %d, last: %d)",
                     msg->sequence, last_received_sequence_);
        return;
    }

    // Update sequence number
    last_received_sequence_ = msg->sequence;

    // Update mission tracking
    current_mission_id_ = msg->mission_id;

    // Update start position if this is the first command or if position changed significantly
    if (!start_position_received_) {
        Eigen::Vector3d new_start_pos(
            msg->start_position.x,
            msg->start_position.y,
            msg->start_position.z
        );

        offset_pt_ = new_start_pos;
        start_pt_ = new_start_pos;
        current_pos_ = new_start_pos;
        start_position_received_ = true;

        FSM_LOG_INFO("Received start position from TrajectoryCommand: (%.2f, %.2f, %.2f)",
                    new_start_pos.x(), new_start_pos.y(), new_start_pos.z());
    }

    FSM_LOG_INFO("Trajectory command (seq: %d): mission=%s, start=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
                msg->sequence,
                msg->mission_id.c_str(),
                msg->start_position.x, msg->start_position.y, msg->start_position.z,
                msg->target_position.x, msg->target_position.y, msg->target_position.z);

    // Extract waypoints
    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& wp : msg->waypoints) {
        waypoints.emplace_back(wp.x, wp.y, wp.z);
    }

    // Extract target position (already calculated by formation_manager)
    Eigen::Vector3d target_position(
        msg->target_position.x,
        msg->target_position.y,
        msg->target_position.z
    );

    // Extract formation offset (for reference/logging)
    Eigen::Vector3d formation_offset(
        msg->formation_offset.x,
        msg->formation_offset.y,
        msg->formation_offset.z
    );

    // Extract full formation pattern from message
    std::vector<Eigen::Vector3d> formation_pattern;
    formation_pattern.reserve(msg->formation_pattern.size());
    for (const auto& pt : msg->formation_pattern) {
        formation_pattern.emplace_back(pt.x, pt.y, pt.z);
    }

    // Store formation pattern for use in formationTargetCallback
    current_formation_pattern_ = formation_pattern;

    // Update formation parameters (for compatibility with existing code)
    current_formation_type_ = msg->formation_type;
    current_formation_scale_ = msg->formation_scale;

    // Set nonholonomic weight based on formation type
    bool is_none_mode = (msg->formation_type == "none" || msg->formation_type == "NONE");
    bool is_line_formation = (msg->formation_type.find("line") != std::string::npos);

    if (is_none_mode || is_line_formation) {
        pending_weight_nonholonomic_ = 0.0;
        FSM_LOG_INFO("Formation mode %s - nonholonomic constraints DISABLED (weight=0)",
                     msg->formation_type.c_str());
        if (path_manager_ && path_manager_->isOptimizerInitialized()) {
            path_manager_->setNonholonomicWeight(0.0);
        }
    } else {
        pending_weight_nonholonomic_ = weight_nonholonomic_;
        FSM_LOG_INFO("Formation mode %s - nonholonomic constraints ENABLED (weight=%.1f)",
                     msg->formation_type.c_str(), weight_nonholonomic_);
        if (path_manager_ && path_manager_->isOptimizerInitialized()) {
            path_manager_->setNonholonomicWeight(weight_nonholonomic_);
        }
    }

    has_formation_command_ = true;

    FSM_LOG_INFO("Drone %d: target=(%.2f, %.2f, %.2f), offset=(%.2f, %.2f, %.2f), %zu waypoints",
                drone_id_,
                target_position.x(), target_position.y(), target_position.z(),
                formation_offset.x(), formation_offset.y(), formation_offset.z(),
                waypoints.size());

    // Publish to formationTargetCallback (which will trigger trajectory planning)
    publishFormationTarget(target_position, waypoints, false, formation_offset);

    auto callback_end = std::chrono::high_resolution_clock::now();
    auto callback_duration = std::chrono::duration_cast<std::chrono::milliseconds>(callback_end - callback_start).count();
    FSM_LOG_INFO("[TRAJECTORY CMD] Callback completed (took %ld ms) at time %.3f",
                 callback_duration, rclcpp::Clock(RCL_ROS_TIME).now().seconds());
}

// generateFormationTargets and generateFormationPattern functions removed
// Formation generation is now handled by formation_manager package

void ReplanFSM::publishFormationTarget(const Eigen::Vector3d& target, const std::vector<Eigen::Vector3d>& waypoints, bool formation_changed, const Eigen::Vector3d& formation_offset) {
    path_manager::msg::FormationTarget target_msg;

    target_msg.header.stamp = node_->now();
    target_msg.header.frame_id = "world";
    target_msg.drone_id = drone_id_;

    target_msg.target_position.x = target.x();
    target_msg.target_position.y = target.y();
    target_msg.target_position.z = target.z();

    target_msg.target_velocity.x = 0.0;
    target_msg.target_velocity.y = 0.0;
    target_msg.target_velocity.z = 0.0;

    target_msg.formation_type = current_formation_type_;
    target_msg.formation_scale = current_formation_scale_;

    // Store formation offset for next transition
    target_msg.formation_offset.x = formation_offset.x();
    target_msg.formation_offset.y = formation_offset.y();
    target_msg.formation_offset.z = formation_offset.z();

    if (!waypoints.empty()) {
        // Simply pass waypoints as-is without adding offset
        // The waypoints from formation commander are already center points
        target_msg.formation_positions.reserve(waypoints.size());
        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point waypoint_pos;
            waypoint_pos.x = wp.x();
            waypoint_pos.y = wp.y();
            waypoint_pos.z = wp.z();
            target_msg.formation_positions.push_back(waypoint_pos);
        }

        log_manager_->infof("Publishing formation target with %zu waypoints for drone %d (no offset applied)",
                   waypoints.size(), drone_id_);
    } else {
        // No waypoints: use current formation center as single waypoint
        geometry_msgs::msg::Point formation_pos;
        formation_pos.x = current_formation_center_.x();
        formation_pos.y = current_formation_center_.y();
        formation_pos.z = current_formation_center_.z();
        target_msg.formation_positions.push_back(formation_pos);

        log_manager_->infof("Publishing formation target with formation center for drone %d",
                   drone_id_);
    }

    formation_target_pub_->publish(target_msg);

}


}  // namespace path_manager
