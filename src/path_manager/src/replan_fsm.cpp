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
      prev_formation_type_(""),  // Empty string indicates no previous formation
      current_formation_scale_(2.0),
      has_formation_command_(false)
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

    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/replan_trajectory_time", -1.0);
    node_->declare_parameter("fsm/n_seconds_ahead", -1.0);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/replan_trajectory_time", replan_trajectory_time_);
    node_->get_parameter("fsm/n_seconds_ahead", n_seconds_ahead_);

    node_->declare_parameter("start_point_x", 0.0);
    node_->declare_parameter("start_point_y", 0.0);
    node_->declare_parameter("start_point_z", 0.0);
    double start_x, start_y, start_z;
    node_->get_parameter("start_point_x", start_x);
    node_->get_parameter("start_point_y", start_y);
    node_->get_parameter("start_point_z", start_z);

    RCLCPP_INFO(node_->get_logger(), "ReplanFSM parameters:");
    log_manager_->infof("ReplanFSM parameters:");
    RCLCPP_INFO(node_->get_logger(), "  start_point_x: %f", start_x);
    log_manager_->infof("  start_point_x: %f", start_x);
    RCLCPP_INFO(node_->get_logger(), "  start_point_y: %f", start_y);
    log_manager_->infof("  start_point_y: %f", start_y);
    RCLCPP_INFO(node_->get_logger(), "  start_point_z: %f", start_z);
    log_manager_->infof("  start_point_z: %f", start_z);
    RCLCPP_INFO(node_->get_logger(), "  Waiting for formation target...");
    log_manager_->infof("  Waiting for formation target...");

    offset_pt_ = Eigen::Vector3d(start_x, start_y, start_z);
    start_pt_ = offset_pt_;
    current_pos_ = offset_pt_;  // Initialize current_pos_ with start position to break circular dependency
    current_vel_ = Eigen::Vector3d::Zero();  // Initialize velocity to zero
    end_pt_ = Eigen::Vector3d::Zero();  // Initialize end_pt_ to avoid uninitialized access
    prev_end_pt_ = Eigen::Vector3d::Zero();  // Initialize previous endpoint
    prev_formation_offset_ = Eigen::Vector3d::Zero();  // Initialize previous formation offset
    have_target_ = false;  // Wait for formation target
    
    RCLCPP_INFO(node_->get_logger(), "Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));
    log_manager_->infof("Initial position set to: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));

    // Initialize PathManager in constructor to avoid nullptr access
    path_manager_ = std::make_shared<PathManager>(node_);
    
    // Initialize SwarmGraph for formation management
    swarm_graph_ = std::make_unique<SwarmGraph>();

    // Initialize swarm_positions_ map for all drones
    for (int i = 0; i < num_drones_; ++i) {
        swarm_positions_[i] = Eigen::Vector3d::Zero();  // Will be updated from broadcast
    }
    FSM_LOG_INFO("Initialized swarm_positions_ for %d drones", num_drones_);

    RCLCPP_INFO(node_->get_logger(), "PathManager initialized, waiting for formation command from formation_commander");
    log_manager_->infof("PathManager initialized, waiting for formation command from formation_commander");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odom_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string odom_topic = "/vehicle" + std::to_string(drone_id_+1) + "/target_position";
    std::string topic_prefix = "/V" + std::to_string(drone_id_+1);    

    optimized_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>("planning/trajectory", sensor_qos);
    global_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>("planning/global", sensor_qos);
    broadcast_traj_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(topic_prefix + "/planning/broadcast_traj_send", sensor_qos);

    if (rviz_simulation_)
    {
        // Internal agent topic for simulation
        std::string target_position_topic = "/agent" + std::to_string(drone_id_) + "/target_position";
        target_position_sub_ = node_->create_subscription<path_manager::msg::PositionCommand>(
            target_position_topic, sensor_qos, std::bind(&ReplanFSM::targetPositionCallback, this, std::placeholders::_1));
        have_position_ = true;  // We have initial position from parameters
    }
    else
    {
        // External MAVLink topic for real PX4
        std::string px4_position_topic = "/vehicle" + std::to_string(mavlink_id_) + "/fmu/out/vehicle_local_position_v1";
        px4_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_position_topic, sensor_qos, std::bind(&ReplanFSM::PX4positionCallback, this, std::placeholders::_1));
    }

    broadcast_traj_sub_ = node_->create_subscription<path_manager::msg::PolyTraj>(
        topic_prefix + "/j_fi/broadcast_traj_recv", sensor_qos,
        std::bind(&ReplanFSM::recvBroadcastPolyTrajCallback, this, std::placeholders::_1));

    formation_target_sub_ = node_->create_subscription<path_manager::msg::FormationTarget>(
        "formation_targets", sensor_qos,
        std::bind(&ReplanFSM::formationTargetCallback, this, std::placeholders::_1));

    formation_cmd_sub_ = node_->create_subscription<path_manager::msg::FormationCommand>(
        "formation_command", sensor_qos,
        std::bind(&ReplanFSM::formationCommandCallback, this, std::placeholders::_1));

    formation_target_pub_ = node_->create_publisher<path_manager::msg::FormationTarget>(
        "formation_targets", sensor_qos);

    waypoint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "waypoint_markers", 10);

    timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this), timer_callback_group_);
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
            bool success;
            if (flag_replan_astar_)
                success = planFromLocalTraj(true, false);
            else
                success = planFromLocalTraj(false, true);

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
                    RCLCPP_ERROR(node_->get_logger(), "No Replan Thresh");
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

    // ⭐ Update own position for Hungarian assignment (thread-safe)
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        current_pos_ = new_pos;
        swarm_positions_[drone_id_] = new_pos;
    }
}

void ReplanFSM::PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    Eigen::Vector3d new_pos;
    new_pos(0) = msg->x + offset_pt_(0);
    new_pos(1) = msg->y + offset_pt_(1);
    new_pos(2) = offset_pt_(2);

    // ⭐ Update own position for Hungarian assignment (thread-safe)
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        current_pos_ = new_pos;
        swarm_positions_[drone_id_] = new_pos;
        have_position_ = true;
    }
}

void ReplanFSM::recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
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

    // ⭐ Update swarm position for Hungarian assignment
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        swarm_positions_[recv_id] = trajectory.getPos(0.0);
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
    path_manager_->getLocalTarget(start_pt_, end_pt_, local_target_pt_, local_target_vel_, t_to_target_);

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

    bool plan_success = path_manager_-> computeAndOptimizePath( 
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, use_formation, have_local_traj_);

    have_new_target_ = false;

    if (enable_global_trajectory_pub_) {
        path_manager::msg::PolyTraj msg2;
        globalTraj2ROSMsg(msg2);
        global_path_pub_->publish(msg2);
    }

    if (plan_success) {
        path_manager::msg::PolyTraj msg;
        polyTraj2ROSMsg(msg);

        optimized_path_pub_->publish(msg);
        broadcast_traj_pub_->publish(msg);
        have_local_traj_ = true;
    }

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

    // start_pt_ = current_pos_;
    // RCLCPP_ERROR(node_->get_logger(), "start_pt_: %.2f, %.2f, %.2f", start_pt_(0), start_pt_(1), start_pt_(2));
    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    double t_ahead = std::min(t_cur + n_seconds_ahead_, info->duration);
    Eigen::Vector3d desired_start_pt = info->traj.getPos(t_cur);

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

    RCLCPP_INFO(node_->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
    log_manager_->infof("[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
    exec_state_ = new_state;
}

void ReplanFSM::formationTargetCallback(const path_manager::msg::FormationTarget::SharedPtr msg) {
    if (msg->drone_id != drone_id_) {
        return;
    }

    if (msg->target_position.z < -0.1) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Formation Target Triggered for drone %d!", drone_id_);
    log_manager_->infof("Formation Target Triggered for drone %d!", drone_id_);

    // Initialize optimizer if not already initialized
    if (!path_manager_->isOptimizerInitialized()) {
        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", drone_id_);
            log_manager_->infof("Initializing optimizer for drone %d...", drone_id_);
            path_manager_->initOptimizer();
            path_manager_->deliverTrajToOptimizer();
            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", drone_id_);
            log_manager_->infof("Optimizer initialized successfully for drone %d", drone_id_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            log_manager_->errorf("Failed to initialize optimizer for drone %d: %s", drone_id_, e.what());
            return;
        }
    }

    start_pt_ = current_pos_;

    // Note: Alpha increment is now handled in computeAndPublishPaths() timer callback
    // This ensures smooth, continuous transitions regardless of when formationTargetCallback is called

    bool success = false;

    std::vector<Eigen::Vector3d> waypoints;

    // ⭐ Waypoints are already calculated with offsets in formationCommandCallback
    // Just use them directly - no need to recalculate or add intermediate points
    if (!msg->formation_positions.empty()) {
        // Simply extract waypoints from message (already have offsets applied)
        for (const auto& pos : msg->formation_positions) {
            waypoints.emplace_back(pos.x, pos.y, pos.z);
        }

        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d: Using %zu waypoints from formation_positions (already offset-applied)",
                   drone_id_, waypoints.size());
        log_manager_->infof(
                   "Drone %d: Using %zu waypoints from formation_positions (already offset-applied)",
                   drone_id_, waypoints.size());

        end_pt_ = waypoints.back();
    } else {
        // No formation_positions: use target_position directly
        // (target_position already has offset applied from formationCommandCallback)
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

    RCLCPP_INFO(node_->get_logger(), "start_pt_: %.2f, %.2f, %.2f", start_pt_(0), start_pt_(1), start_pt_(2));
    RCLCPP_INFO(node_->get_logger(), "waypoints size: %zu", waypoints.size());
    for (const auto& wp : waypoints) {
        RCLCPP_INFO(node_->get_logger(), "waypoint: %.2f, %.2f, %.2f", wp(0), wp(1), wp(2));
    }

    // Visualize waypoints in RViz (optional)
    if (enable_waypoint_markers_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints_drone_" + std::to_string(drone_id_);
        marker.id = drone_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;  // Sphere size

        // Set color based on drone_id
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

        // Add all waypoints
        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = wp(0);
            p.y = wp(1);
            p.z = wp(2);
            marker.points.push_back(p);
        }

        waypoint_marker_pub_->publish(marker);
        RCLCPP_INFO(node_->get_logger(), "Drone %d: Published %zu waypoint markers to RViz (frame: %s, ns: %s)",
                    drone_id_, waypoints.size(), marker.header.frame_id.c_str(), marker.ns.c_str());
    }

    // Set formation info to PathManager for outer/inner line calculation
    std::vector<Eigen::Vector3d> formation_pattern =
        generateFormationPattern(current_formation_type_, num_drones_, current_formation_scale_);
    path_manager_->setFormationInfo(drone_id_, current_formation_type_, formation_pattern);

    success = path_manager_->planGlobalTraj(
        current_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success) {
        // Set current formation pattern to optimizer (immediate change)
        FSM_LOG_INFO("Setting formation to optimizer: %s", current_formation_type_.c_str());

        RCLCPP_INFO(node_->get_logger(), "Formation pattern for optimizer (relative coordinates):");
        log_manager_->infof("Formation pattern for optimizer (relative coordinates):");
        for (size_t i = 0; i < formation_pattern.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "  Drone %zu: [%.3f, %.3f, %.3f]",
                       i, formation_pattern[i].x(), formation_pattern[i].y(), formation_pattern[i].z());
            log_manager_->infof("  Drone %zu: [%.3f, %.3f, %.3f]",
                               i, formation_pattern[i].x(), formation_pattern[i].y(), formation_pattern[i].z());
        }

        // Pass formation pattern to optimizer
        path_manager_->setFormationToOptimizer(formation_pattern, formation_pattern.size());

        // Save current endpoint and offset for next formation transition
        prev_end_pt_ = end_pt_;
        prev_formation_offset_ = Eigen::Vector3d(
            msg->formation_offset.x,
            msg->formation_offset.y,
            msg->formation_offset.z
        );
        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d: Saved endpoint (%.2f, %.2f, %.2f) and offset (%.2f, %.2f, %.2f) for next transition",
                   drone_id_,
                   prev_end_pt_.x(), prev_end_pt_.y(), prev_end_pt_.z(),
                   prev_formation_offset_.x(), prev_formation_offset_.y(), prev_formation_offset_.z());
        log_manager_->infof(
                   "Drone %d: Saved endpoint and offset for next transition",
                   drone_id_);

        have_target_ = true;
        have_new_target_ = true;

        if (exec_state_ == WAIT_POSITION)
            changeFSMExecState(SEQUENTIAL_START, "formationTargetCallback");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "formationTargetCallback");

        if (enable_global_trajectory_pub_) {
            path_manager::msg::PolyTraj traj_msg;
            globalTraj2ROSMsg(traj_msg);
            global_path_pub_->publish(traj_msg);
        }

        RCLCPP_INFO(node_->get_logger(), "Successfully generated%s global trajectory for drone %d",
                    enable_global_trajectory_pub_ ? " and published" : "", drone_id_);
        log_manager_->infof("Successfully generated and published global trajectory for drone %d", drone_id_);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory for drone %d!", drone_id_);
        log_manager_->errorf("Unable to generate global trajectory for drone %d!", drone_id_);
    }
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

    return true;
}

void ReplanFSM::formationCommandCallback(const path_manager::msg::FormationCommand::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(),
                "Received formation command: %s, scale: %.2f, center: (%.2f, %.2f, %.2f)",
                msg->formation_type.c_str(),
                msg->formation_scale,
                msg->formation_center.x,
                msg->formation_center.y,
                msg->formation_center.z);

    // Detect formation change
    bool formation_changed = (!prev_formation_type_.empty() &&
                             prev_formation_type_ != msg->formation_type);

    if (formation_changed) {
        FSM_LOG_INFO("Formation change detected: %s -> %s (immediate transition)",
                   prev_formation_type_.c_str(),
                   msg->formation_type.c_str());
    }

    // Update formation parameters
    prev_formation_type_ = current_formation_type_;  // Save current before updating
    current_formation_type_ = msg->formation_type;
    current_formation_scale_ = msg->formation_scale;
    current_formation_center_ = Eigen::Vector3d(
        msg->formation_center.x,
        msg->formation_center.y,
        msg->formation_center.z
    );

    has_formation_command_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& wp : msg->waypoints) {
        waypoints.emplace_back(wp.x, wp.y, wp.z);
    }

    // === HUNGARIAN ALGORITHM FOR OPTIMAL ASSIGNMENT ===

    // Generate formation pattern (target positions)
    std::vector<Eigen::Vector3d> formation_pattern =
        generateFormationPattern(current_formation_type_, num_drones_, current_formation_scale_);

    // Collect current positions of all drones
    std::vector<Eigen::Vector3d> current_positions(num_drones_);
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        bool all_positions_available = true;
        for (int i = 0; i < num_drones_; ++i) {
            if (swarm_positions_.find(i) != swarm_positions_.end()) {
                current_positions[i] = swarm_positions_[i];
            } else {
                all_positions_available = false;
                RCLCPP_WARN(node_->get_logger(),
                           "Position for drone %d not available yet", i);
            }
        }

        if (!all_positions_available) {
            // Fallback to ID-based assignment if not all positions available
            RCLCPP_WARN(node_->get_logger(),
                       "Not all drone positions available, using ID-based assignment");
            std::vector<Eigen::Vector3d> target_positions;
            target_positions.reserve(num_drones_);
            for (const auto& pattern_point : formation_pattern) {
                target_positions.push_back(current_formation_center_ + pattern_point);
            }
            Eigen::Vector3d my_target = target_positions[drone_id_];
            publishFormationTarget(my_target, waypoints, formation_changed);
            return;
        }
    }

    // Calculate intermediate formation center for Hungarian assignment
    // This ensures smooth formation changes based on current swarm position
    Eigen::Vector3d prev_swarm_center = Eigen::Vector3d::Zero();
    for (int i = 0; i < num_drones_; ++i) {
        prev_swarm_center += current_positions[i];
    }
    prev_swarm_center /= num_drones_;

    Eigen::Vector3d first_pure_waypoint;
    if (!waypoints.empty()) {
        first_pure_waypoint = waypoints[0];  // First waypoint (pure, no offset)
    } else {
        first_pure_waypoint = current_formation_center_;
    }

    // Intermediate formation center = midpoint between prev swarm center and first pure waypoint
    Eigen::Vector3d intermediate_formation_center = (prev_swarm_center + first_pure_waypoint) / 2.0;

    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] Prev swarm center: (%.2f, %.2f, %.2f)",
               prev_swarm_center.x(), prev_swarm_center.y(), prev_swarm_center.z());
    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] First pure waypoint: (%.2f, %.2f, %.2f)",
               first_pure_waypoint.x(), first_pure_waypoint.y(), first_pure_waypoint.z());
    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] Intermediate formation center: (%.2f, %.2f, %.2f)",
               intermediate_formation_center.x(), intermediate_formation_center.y(), intermediate_formation_center.z());

    // Create intermediate formation positions (for Hungarian assignment)
    std::vector<Eigen::Vector3d> intermediate_formation_positions;
    intermediate_formation_positions.reserve(num_drones_);
    for (const auto& pattern_point : formation_pattern) {
        intermediate_formation_positions.push_back(intermediate_formation_center + pattern_point);
    }

    // Run Hungarian algorithm to find optimal assignment BASED ON INTERMEDIATE POSITIONS
    std::vector<int> assignment = hungarianAssignment(current_positions, intermediate_formation_positions, waypoints);

    // Extract offset from intermediate assignment
    Eigen::Vector3d my_intermediate_position = intermediate_formation_positions[assignment[drone_id_]];
    Eigen::Vector3d my_formation_offset = my_intermediate_position - intermediate_formation_center;

    RCLCPP_INFO(node_->get_logger(),
               "Drone %d assigned to intermediate position %d: (%.2f, %.2f, %.2f)",
               drone_id_, assignment[drone_id_],
               my_intermediate_position.x(), my_intermediate_position.y(), my_intermediate_position.z());
    RCLCPP_INFO(node_->get_logger(),
               "Drone %d formation offset: (%.2f, %.2f, %.2f)",
               drone_id_, my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());

    // Build waypoints: Apply path-normal offset for line formations
    std::vector<Eigen::Vector3d> my_waypoints;

    RCLCPP_INFO(node_->get_logger(),
               "Drone %d: Base formation offset: (%.2f, %.2f, %.2f)",
               drone_id_, my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());

    // For line formations, apply offset in the normal direction of the path
    bool use_path_normal = (current_formation_type_ == "line_first" ||
                            current_formation_type_ == "line_second" ||
                            current_formation_type_ == "line_first_no_offset" ||
                            current_formation_type_ == "line_second_no_offset");

    // Calculate the signed offset distance (perpendicular distance from centerline)
    // Determine which side of the line this drone should be on
    double offset_distance = my_formation_offset.norm();

    // Calculate reference normal direction at intermediate point
    Eigen::Vector3d ref_tangent(1.0, 0.0, 0.0);
    if (waypoints.size() >= 2) {
        ref_tangent = (waypoints[0] - intermediate_formation_center).normalized();
    }
    // Normal is perpendicular to tangent (rotate 90° right in 2D: (x,y) -> (y,-x))
    Eigen::Vector3d ref_normal(ref_tangent.y(), -ref_tangent.x(), 0.0);

    // Determine sign: which side of the line?
    // If offset has positive component in ref_normal direction, keep positive
    double offset_sign = (my_formation_offset.dot(ref_normal) >= 0) ? 1.0 : -1.0;
    offset_distance *= offset_sign;

    if (use_path_normal) {
        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d: Using path-normal offset (distance=%.2fm, sign=%.0f)",
                   drone_id_, std::abs(offset_distance), offset_sign);
    }

    // Add all waypoints with appropriate offset
    for (size_t i = 0; i < waypoints.size(); ++i) {
        Eigen::Vector3d applied_offset = my_formation_offset;  // Default: fixed offset

        if (use_path_normal && std::abs(offset_distance) > 1e-6) {
            // Calculate path tangent (direction) at this waypoint
            Eigen::Vector3d path_tangent(1.0, 0.0, 0.0);

            if (i == 0 && waypoints.size() >= 2) {
                // First waypoint: use direction to next waypoint
                path_tangent = (waypoints[1] - waypoints[0]).normalized();
            } else if (i == waypoints.size() - 1 && i > 0) {
                // Last waypoint: use direction from previous waypoint
                path_tangent = (waypoints[i] - waypoints[i-1]).normalized();
            } else if (i > 0 && i < waypoints.size() - 1) {
                // Middle waypoints: use average of incoming and outgoing directions
                Eigen::Vector3d incoming = (waypoints[i] - waypoints[i-1]).normalized();
                Eigen::Vector3d outgoing = (waypoints[i+1] - waypoints[i]).normalized();
                path_tangent = ((incoming + outgoing) / 2.0).normalized();
            }

            // Calculate path normal (perpendicular to tangent, pointing right in 2D)
            // For 2D: if tangent = (tx, ty), normal = (ty, -tx)
            Eigen::Vector3d path_normal(path_tangent.y(), -path_tangent.x(), 0.0);

            // Apply offset in the normal direction
            applied_offset = path_normal * offset_distance;

            RCLCPP_INFO(node_->get_logger(),
                       "Drone %d WP%zu: tangent=(%.3f,%.3f) normal=(%.3f,%.3f) offset=(%.2f,%.2f)",
                       drone_id_, i,
                       path_tangent.x(), path_tangent.y(),
                       path_normal.x(), path_normal.y(),
                       applied_offset.x(), applied_offset.y());
        }

        Eigen::Vector3d wp_with_offset = waypoints[i] + applied_offset;
        my_waypoints.push_back(wp_with_offset);

        RCLCPP_INFO(node_->get_logger(),
                   "Drone %d WP%zu: (%.2f, %.2f) + offset = (%.2f, %.2f)",
                   drone_id_, i,
                   waypoints[i].x(), waypoints[i].y(),
                   wp_with_offset.x(), wp_with_offset.y());
    }

    // Final target is last waypoint + offset
    Eigen::Vector3d my_target;
    if (!my_waypoints.empty()) {
        my_target = my_waypoints.back();
    } else {
        my_target = current_formation_center_ + my_formation_offset;
    }

    RCLCPP_INFO(node_->get_logger(),
               "Drone %d final target: (%.2f, %.2f, %.2f) with %zu waypoints",
               drone_id_, my_target.x(), my_target.y(), my_target.z(), my_waypoints.size());

    // Publish formation target with my waypoints and offset
    publishFormationTarget(my_target, my_waypoints, formation_changed, my_formation_offset);
}

void ReplanFSM::generateFormationTargets(
    const Eigen::Vector3d& center, 
    const std::string& formation_type, 
    double scale,
    const std::vector<Eigen::Vector3d>& waypoints)
{
    // Use relative coordinates for formation pattern
    std::vector<Eigen::Vector3d> formation_pattern =
        generateFormationPattern(formation_type, num_drones_, scale);

    if (swarm_graph_) {
        swarm_graph_->setDesiredForm(formation_pattern);  // Pass relative coordinates
        RCLCPP_INFO(node_->get_logger(), "Set desired formation in SwarmGraph: %s", formation_type.c_str());
    }

    // For publishFormationTarget, use absolute coordinates (center + pattern)
    Eigen::Vector3d my_target = center + formation_pattern[drone_id_];
    publishFormationTarget(my_target, waypoints);
}

std::vector<Eigen::Vector3d> ReplanFSM::generateFormationPattern(
    const std::string& formation_type, int num_drones, double scale)
{
    std::vector<Eigen::Vector3d> pattern;
    pattern.reserve(num_drones);

    if (formation_type == "square" && num_drones == 4) {
        // Standard square formation
        pattern.push_back(Eigen::Vector3d(-scale/2, -scale/2, 0.0));  // Drone 0: 왼쪽 아래
        pattern.push_back(Eigen::Vector3d(-scale/2,  scale/2, 0.0));  // Drone 1: 왼쪽 위
        pattern.push_back(Eigen::Vector3d( scale/2,  scale/2, 0.0));  // Drone 2: 오른쪽 위
        pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));  // Drone 3: 오른쪽 아래
    }
    else if (formation_type == "triangle" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Triangle vertices: each drone at a corner
        pattern.push_back(Eigen::Vector3d(0.0, 2.0*h/3.0, 0.0));          // Top (apex)
        pattern.push_back(Eigen::Vector3d(-scale/2.0, -h/3.0, 0.0));      // Bottom left
        pattern.push_back(Eigen::Vector3d(+scale/2.0, -h/3.0, 0.0));      // Bottom right

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));            // Center
        }
    }
    else if (formation_type == "triangle_rotated" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Rotated 90 degrees clockwise: apex points left (direction of travel)
        pattern.push_back(Eigen::Vector3d(-2.0*h/3.0, 0.0, 0.0));         // Left center (apex, direction of travel)
        pattern.push_back(Eigen::Vector3d(+h/3.0, +scale/2.0, 0.0));      // Right top
        pattern.push_back(Eigen::Vector3d(+h/3.0, 0.0, 0.0));             // Right center

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(+h/3.0, -scale/2.0, 0.0));  // Right bottom
        }
    }
    
    // TODO: Add other formation types
    
    // else if (formation_type == "Echelon" && num_drones >= 2) {
    //     int sign = +1; 
    //     double dx = 0.5 * scale;
    //     double dy = 0.6 * scale;
    
    //     pattern.clear();
    //     for (int i = 0; i < num_drones; ++i) {
    //         double x = sign * i * dx;
    //         double y = - i * dy;
    //         pattern.push_back(Eigen::Vector3d(x, y, 0.0));
    //     }
    // }
    // else if (formation_type == "Staggered" && num_drones >= 2) {
    //     double dx = 0.5 * scale;
    //     double dy = 0.6 * scale;
    
    //     pattern.clear();
    //     for (int i = 0; i < num_drones; ++i) {
    //         int side = (i % 2 == 0) ? -1 : +1;
    //         double x = side * dx;
    //         double y = - (i/2) * dy;
    //         pattern.push_back(Eigen::Vector3d(x, y, 0.0));
    //     }
    // }
    else if (formation_type == "line_first") {
        double spacing = (num_drones > 1) ? scale / (num_drones - 1) : 0.0;
        double line_angle = 83.0 * M_PI / 180.0;

        for (int i = 0; i < num_drones; ++i) {
            double line_position = -scale/2 + i * spacing;
            pattern.push_back(Eigen::Vector3d(
                line_position * cos(line_angle),
                line_position * sin(line_angle),
                0.0
            ));
        }
    }
    else if (formation_type == "line_first_no_offset") {
        // No offset version - all drones target same waypoint, formation maintained by local optimizer
        for (int i = 0; i < num_drones; ++i) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));  // Zero offset for all
        }
    }
    else if (formation_type == "line_second") {
        double spacing = (num_drones > 1) ? scale / (num_drones - 1) : 0.0;
        double line_angle = -8.63 * M_PI / 180.0;

        for (int i = 0; i < num_drones; ++i) {
            double line_position = -scale/2 + i * spacing;
            pattern.push_back(Eigen::Vector3d(
                line_position * cos(line_angle),
                line_position * sin(line_angle),
                0.0
            ));
        }
    }
    else if (formation_type == "line_second_no_offset") {
        // No offset version - all drones target same waypoint, formation maintained by local optimizer
        for (int i = 0; i < num_drones; ++i) {
            pattern.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));  // Zero offset for all
        }
    }
    else if (formation_type == "circle") {
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
        RCLCPP_INFO(node_->get_logger(),
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

        RCLCPP_INFO(node_->get_logger(),
                   "Publishing formation target with %zu waypoints for drone %d (no offset applied)",
                   waypoints.size(), drone_id_);
    } else {
        // No waypoints: use current formation center as single waypoint
        geometry_msgs::msg::Point formation_pos;
        formation_pos.x = current_formation_center_.x();
        formation_pos.y = current_formation_center_.y();
        formation_pos.z = current_formation_center_.z();
        target_msg.formation_positions.push_back(formation_pos);

        RCLCPP_INFO(node_->get_logger(),
                   "Publishing formation target with formation center for drone %d",
                   drone_id_);
    }

    formation_target_pub_->publish(target_msg);

}

// Hungarian algorithm implementation for optimal assignment
std::vector<int> ReplanFSM::hungarianAssignment(
    const std::vector<Eigen::Vector3d>& current_positions,
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Vector3d>& waypoints)
{
    int n = current_positions.size();
    if (n != target_positions.size()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Hungarian: Size mismatch! current=%d, target=%d",
                     n, (int)target_positions.size());
        // Fallback to identity assignment
        std::vector<int> assignment(n);
        std::iota(assignment.begin(), assignment.end(), 0);
        return assignment;
    }

    // Build cost matrix with multi-factor cost function
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n));

    // Get formation pattern for orientation calculation
    // Use the current formation type's geometry to preserve orientation
    std::vector<Eigen::Vector3d> formation_pattern =
        generateFormationPattern(current_formation_type_, num_drones_, current_formation_scale_);

    // Compute formation centers
    Eigen::Vector3d current_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        current_center += current_positions[i];
        target_center += target_positions[i];
    }
    current_center /= n;
    target_center /= n;

    // === PATH-AWARE FORMATION ORIENTATION ===
    // Calculate desired formation orientation based on path direction
    Eigen::Vector3d path_direction(1.0, 0.0, 0.0);  // Default: forward (X-axis)
    bool has_valid_direction = false;

    if (waypoints.size() >= 2) {
        // Find the waypoint segment closest to target center
        double min_dist = std::numeric_limits<double>::infinity();
        int closest_segment = -1;

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            Eigen::Vector3d segment_start = waypoints[i];
            Eigen::Vector3d segment_end = waypoints[i + 1];
            Eigen::Vector3d segment_mid = (segment_start + segment_end) / 2.0;

            double dist = (segment_mid - target_center).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_segment = i;
            }
        }

        if (closest_segment >= 0) {
            // Calculate path tangent direction at this segment
            Eigen::Vector3d tangent = waypoints[closest_segment + 1] - waypoints[closest_segment];
            double tangent_norm = tangent.norm();

            if (tangent_norm > 1e-6) {
                path_direction = tangent / tangent_norm;  // Normalize
                has_valid_direction = true;

                RCLCPP_INFO(node_->get_logger(),
                           "[HUNGARIAN] Path direction at target: (%.3f, %.3f, %.3f) from waypoints %d-%d",
                           path_direction.x(), path_direction.y(), path_direction.z(),
                           closest_segment, closest_segment + 1);
                log_manager_->infof("[HUNGARIAN] Path direction: (%.3f, %.3f, %.3f)",
                                   path_direction.x(), path_direction.y(), path_direction.z());
            }
        }
    }

    // Rotate formation pattern to align with path direction
    // Formation pattern is defined with forward direction as +X axis
    // We need to rotate it to align with actual path direction
    std::vector<Eigen::Vector3d> rotated_pattern = formation_pattern;

    if (has_valid_direction) {
        // Calculate rotation angle from default direction (1,0,0) to path direction
        Eigen::Vector3d default_dir(1.0, 0.0, 0.0);

        // Only consider XY plane rotation (ignore Z for 2D formations)
        Eigen::Vector2d default_dir_2d(default_dir.x(), default_dir.y());
        Eigen::Vector2d path_dir_2d(path_direction.x(), path_direction.y());

        double default_norm = default_dir_2d.norm();
        double path_norm = path_dir_2d.norm();

        if (default_norm > 1e-6 && path_norm > 1e-6) {
            // Calculate rotation angle using atan2 for proper quadrant handling
            double angle_default = std::atan2(default_dir_2d.y(), default_dir_2d.x());
            double angle_path = std::atan2(path_dir_2d.y(), path_dir_2d.x());
            double rotation_angle = angle_path - angle_default;

            // Rotation matrix for Z-axis rotation (2D rotation in XY plane)
            double cos_theta = std::cos(rotation_angle);
            double sin_theta = std::sin(rotation_angle);

            for (size_t i = 0; i < formation_pattern.size(); ++i) {
                Eigen::Vector3d original = formation_pattern[i];

                // Apply 2D rotation in XY plane
                rotated_pattern[i] = Eigen::Vector3d(
                    cos_theta * original.x() - sin_theta * original.y(),
                    sin_theta * original.x() + cos_theta * original.y(),
                    original.z()  // Z unchanged
                );
            }

            RCLCPP_INFO(node_->get_logger(),
                       "[HUNGARIAN] Formation pattern rotated by %.2f degrees to align with path",
                       rotation_angle * 180.0 / M_PI);
            log_manager_->infof("[HUNGARIAN] Pattern rotated by %.2f deg",
                               rotation_angle * 180.0 / M_PI);
        }
    }

    // === ORDER-PRESERVING ASSIGNMENT FOR LINE FORMATIONS ===
    // For line formations, preserve ordering along the line direction
    // This prevents drones from crossing paths and maintains stable formation
    if (current_formation_type_ == "line_first" || current_formation_type_ == "line_second" ||
        current_formation_type_ == "line_first_no_offset" || current_formation_type_ == "line_second_no_offset") {

        RCLCPP_INFO(node_->get_logger(),
                   "[ASSIGNMENT] Using order-preserving assignment for line formation: %s",
                   current_formation_type_.c_str());
        log_manager_->infof("[ASSIGNMENT] Order-preserving for line formation: %s",
                           current_formation_type_.c_str());

        // Calculate line direction from formation pattern
        Eigen::Vector3d line_direction(1.0, 0.0, 0.0);  // Default
        if (rotated_pattern.size() >= 2) {
            line_direction = (rotated_pattern.back() - rotated_pattern.front()).normalized();
        }

        // Project current positions onto line direction and sort
        std::vector<std::pair<double, int>> current_projections;
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d relative_pos = current_positions[i] - current_center;
            double projection = relative_pos.dot(line_direction);
            current_projections.push_back({projection, i});
        }
        std::sort(current_projections.begin(), current_projections.end());

        // Project target positions onto line direction and sort
        std::vector<std::pair<double, int>> target_projections;
        for (int j = 0; j < n; ++j) {
            Eigen::Vector3d relative_pos = target_positions[j] - target_center;
            double projection = relative_pos.dot(line_direction);
            target_projections.push_back({projection, j});
        }
        std::sort(target_projections.begin(), target_projections.end());

        // Create order-preserving assignment: i-th drone along line -> i-th target along line
        std::vector<int> assignment(n);
        for (int i = 0; i < n; ++i) {
            int drone_idx = current_projections[i].second;
            int target_idx = target_projections[i].second;
            assignment[drone_idx] = target_idx;

            RCLCPP_INFO(node_->get_logger(),
                       "[ASSIGNMENT] Drone %d (proj=%.2f) -> Target %d (proj=%.2f)",
                       drone_idx, current_projections[i].first,
                       target_idx, target_projections[i].first);
        }

        return assignment;
    }

    // === HUNGARIAN ALGORITHM FOR OTHER FORMATIONS ===
    // SIMPLIFIED: Only use Euclidean distance between current position and target formation position
    // No complex cost functions - just assign each drone to nearest available formation spot
    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] Using Hungarian algorithm for formation: %s",
               current_formation_type_.c_str());
    log_manager_->infof("[HUNGARIAN] Using Hungarian algorithm for: %s",
                       current_formation_type_.c_str());

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            // Cost = Euclidean distance from current position to target formation position
            cost_matrix[i][j] = (current_positions[i] - target_positions[j]).norm();
        }
    }

    // Debug: Print cost matrix
    RCLCPP_INFO(node_->get_logger(), "[HUNGARIAN] Cost matrix:");
    for (int i = 0; i < n; ++i) {
        std::string row_str = "  Drone " + std::to_string(i) + ": ";
        for (int j = 0; j < n; ++j) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f ", cost_matrix[i][j]);
            row_str += buf;
        }
        RCLCPP_INFO(node_->get_logger(), "%s", row_str.c_str());
        log_manager_->infof("%s", row_str.c_str());
    }

    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] Cost matrix computed with simple Euclidean distance");
    log_manager_->infof("[HUNGARIAN] Cost matrix: simple Euclidean distance only");

    // ===== Hungarian Algorithm Implementation =====
    // This is the O(n^3) Hungarian algorithm (Kuhn-Munkres)

    std::vector<double> u(n + 1), v(n + 1);  // Dual variables
    std::vector<int> p(n + 1), way(n + 1);   // Matching and path

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(n + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            int j1 = 0;

            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    double cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    // Extract assignment: assignment[i] = j means drone i -> target j
    std::vector<int> assignment(n, 0);  // Initialize with default values
    for (int j = 1; j <= n; ++j) {
        if (p[j] != 0) {
            int drone_idx = p[j] - 1;
            int target_idx = j - 1;

            // Bounds check to prevent memory corruption
            if (drone_idx >= 0 && drone_idx < n && target_idx >= 0 && target_idx < n) {
                assignment[drone_idx] = target_idx;
            } else {
                RCLCPP_ERROR(node_->get_logger(),
                            "[HUNGARIAN] Assignment out of bounds: drone=%d, target=%d (n=%d)",
                            drone_idx, target_idx, n);
                log_manager_->errorf("[HUNGARIAN] Assignment out of bounds: drone=%d, target=%d (n=%d)",
                                    drone_idx, target_idx, n);
            }
        }
    }

    // Validate all assignments are within bounds
    for (int i = 0; i < n; ++i) {
        if (assignment[i] < 0 || assignment[i] >= n) {
            RCLCPP_WARN(node_->get_logger(),
                       "[HUNGARIAN] Invalid assignment for drone %d, using identity assignment",
                       i);
            assignment[i] = i;
        }
    }

    // Log assignment and total cost with detailed information
    double total_cost = 0.0;
    RCLCPP_INFO(node_->get_logger(), "[HUNGARIAN] Assignment result:");
    for (int i = 0; i < n; ++i) {
        total_cost += cost_matrix[i][assignment[i]];

        Eigen::Vector3d curr_pos = current_positions[i];
        Eigen::Vector3d tgt_pos = target_positions[assignment[i]];
        double euclidean_dist = (curr_pos - tgt_pos).norm();

        RCLCPP_INFO(node_->get_logger(),
                   "[HUNGARIAN]   Drone %d (%.1f,%.1f) -> Target %d (%.1f,%.1f) | Cost: %.2f | EucDist: %.2f",
                   i, curr_pos.x(), curr_pos.y(),
                   assignment[i], tgt_pos.x(), tgt_pos.y(),
                   cost_matrix[i][assignment[i]], euclidean_dist);
        log_manager_->infof(
                   "[HUNGARIAN]   Drone %d (%.1f,%.1f) -> Target %d (%.1f,%.1f) | Cost: %.2f | EucDist: %.2f",
                   i, curr_pos.x(), curr_pos.y(),
                   assignment[i], tgt_pos.x(), tgt_pos.y(),
                   cost_matrix[i][assignment[i]], euclidean_dist);
    }
    RCLCPP_INFO(node_->get_logger(),
               "[HUNGARIAN] Total assignment cost: %.2f", total_cost);
    log_manager_->infof("[HUNGARIAN] Total assignment cost: %.2f", total_cost);

    return assignment;
}


}  // namespace path_manager
