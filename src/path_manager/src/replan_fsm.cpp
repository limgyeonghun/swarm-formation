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
                
            path_manager::msg::PolyTraj msg;
            globalTraj2ROSMsg(msg);
            global_path_pub_->publish(msg);
            FSM_LOG_INFO("Published global trajectory successfully!");
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
            // TODO: Implement emergency stop logic
            RCLCPP_ERROR(node_->get_logger(), "EMERGENCY_STOP");
            log_manager_->errorf("EMERGENCY_STOP");
            break;
        }
    }
}

void ReplanFSM::targetPositionCallback(const path_manager::msg::PositionCommand::SharedPtr msg) {
    current_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    // ⭐ Update own position for Hungarian assignment
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        swarm_positions_[drone_id_] = current_pos_;
    }
}

void ReplanFSM::PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_pos_(0) = msg->x + offset_pt_(0);
    current_pos_(1) = msg->y + offset_pt_(1);
    current_pos_(2) = offset_pt_(2);
    have_position_ = true;

    // ⭐ Update own position for Hungarian assignment
    {
        std::lock_guard<std::mutex> lock(swarm_positions_mutex_);
        swarm_positions_[drone_id_] = current_pos_;
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

    path_manager::msg::PolyTraj msg2;
    globalTraj2ROSMsg(msg2);
    global_path_pub_->publish(msg2);

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
    double t_debug_start = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
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

    // ⭐ Waypoint calculation: Use current_formation_type (already updated to target)
    // Waypoints should be fixed goals based on the target formation
    std::vector<Eigen::Vector3d> waypoint_pattern =
        generateFormationPattern(current_formation_type_, num_drones_, current_formation_scale_);

    Eigen::Vector3d my_formation_offset = Eigen::Vector3d::Zero();
    if (drone_id_ < waypoint_pattern.size()) {
        my_formation_offset = waypoint_pattern[drone_id_];
    }

    // Build waypoints from formation_positions (center points) + my offset
    if (!msg->formation_positions.empty()) {
        // Check if current formation is a line formation
        bool is_line_formation = (current_formation_type_ == "line_first" ||
                                  current_formation_type_ == "line_second" ||
                                  current_formation_type_ == "line_first_no_offset" ||
                                  current_formation_type_ == "line_second_no_offset");

        if (is_line_formation) {
            // ⭐ Line formation: Use ALL waypoints from formation_positions (no intermediate alignment point)
            for (const auto& pos : msg->formation_positions) {
                Eigen::Vector3d center_point(pos.x, pos.y, pos.z);
                Eigen::Vector3d my_waypoint = center_point + my_formation_offset;
                waypoints.emplace_back(my_waypoint);
            }

            RCLCPP_INFO(node_->get_logger(),
                       "Drone %d: Line formation - using %zu waypoints (no intermediate alignment)",
                       drone_id_, waypoints.size());
            log_manager_->infof(
                       "Drone %d: Line formation - using %zu waypoints (no intermediate alignment)",
                       drone_id_, waypoints.size());
        } else {
            // Non-line formation: Add intermediate alignment point
            Eigen::Vector3d first_target_center(
                msg->formation_positions[0].x,
                msg->formation_positions[0].y,
                msg->formation_positions[0].z
            );

            // Calculate direction and distance from current position to first target
            Eigen::Vector3d direction = first_target_center - current_pos_;
            double total_distance = direction.norm();

            // Place intermediate formation center at a fraction of the distance
            // Use formation scale to determine distance (ensure drones have space to align)
            double alignment_distance = std::min(
                current_formation_scale_ * 2.0,  // At least 2x formation scale
                total_distance * 0.3               // Or 30% of total distance
            );

            Eigen::Vector3d intermediate_formation_center;
            if (total_distance > 1e-3) {
                intermediate_formation_center = current_pos_ + direction.normalized() * alignment_distance;
            } else {
                // If too close, just use current position
                intermediate_formation_center = current_pos_;
            }

            // Add intermediate alignment waypoint (current formation → target formation)
            Eigen::Vector3d my_alignment_waypoint = intermediate_formation_center + my_formation_offset;
            waypoints.emplace_back(my_alignment_waypoint);

            RCLCPP_INFO(node_->get_logger(),
                       "Drone %d: Added intermediate alignment point at (%.2f, %.2f, %.2f), distance=%.2fm from start",
                       drone_id_,
                       my_alignment_waypoint.x(), my_alignment_waypoint.y(), my_alignment_waypoint.z(),
                       alignment_distance);
            log_manager_->infof(
                       "Drone %d: Intermediate formation center at (%.2f, %.2f, %.2f)",
                       drone_id_,
                       intermediate_formation_center.x(), intermediate_formation_center.y(), intermediate_formation_center.z());

            // Add remaining waypoints from formation_positions
            for (const auto& pos : msg->formation_positions) {
                Eigen::Vector3d center_point(pos.x, pos.y, pos.z);
                Eigen::Vector3d my_waypoint = center_point + my_formation_offset;
                waypoints.emplace_back(my_waypoint);
            }

            RCLCPP_INFO(node_->get_logger(),
                       "Using %zu waypoints from formation_positions for drone %d (offset: %.2f, %.2f, %.2f)",
                       waypoints.size(), drone_id_,
                       my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());
            log_manager_->infof(
                       "Using %zu waypoints from formation_positions for drone %d (offset: %.2f, %.2f, %.2f)",
                       waypoints.size(), drone_id_,
                       my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());
        }

        end_pt_ = waypoints.back();
    } else {
        // No formation_positions: use target_position + offset
        Eigen::Vector3d center_pos(
            msg->target_position.x,
            msg->target_position.y,
            msg->target_position.z);
        Eigen::Vector3d my_target = center_pos + my_formation_offset;
        waypoints = { my_target };
        end_pt_ = my_target;
        RCLCPP_INFO(node_->get_logger(),
                   "Using single target_position as waypoint for drone %d (offset: %.2f, %.2f, %.2f)",
                   drone_id_,
                   my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());
        log_manager_->infof(
                   "Using single target_position as waypoint for drone %d (offset: %.2f, %.2f, %.2f)",
                   drone_id_,
                   my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());
    }

    RCLCPP_INFO(node_->get_logger(), "start_pt_: %.2f, %.2f, %.2f", start_pt_(0), start_pt_(1), start_pt_(2));
    RCLCPP_INFO(node_->get_logger(), "waypoints size: %zu", waypoints.size());
    for (const auto& wp : waypoints) {
        RCLCPP_INFO(node_->get_logger(), "waypoint: %.2f, %.2f, %.2f", wp(0), wp(1), wp(2));
    }

    success = path_manager_->planGlobalTraj(
        current_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success) {
        // Set current formation pattern to optimizer (immediate change)
        std::vector<Eigen::Vector3d> formation_pattern =
            generateFormationPattern(current_formation_type_, num_drones_, current_formation_scale_);

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

        have_target_ = true;
        have_new_target_ = true;

        if (exec_state_ == WAIT_POSITION)
            changeFSMExecState(SEQUENTIAL_START, "formationTargetCallback");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "formationTargetCallback");

        path_manager::msg::PolyTraj traj_msg;
        globalTraj2ROSMsg(traj_msg);
        global_path_pub_->publish(traj_msg);
            
        RCLCPP_INFO(node_->get_logger(), "Successfully generated and published global trajectory for drone %d", drone_id_);
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

    std::vector<Eigen::Vector3d> target_positions;
    target_positions.reserve(num_drones_);
    for (const auto& pattern_point : formation_pattern) {
        target_positions.push_back(current_formation_center_ + pattern_point);
    }

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
            Eigen::Vector3d my_target = target_positions[drone_id_];
            publishFormationTarget(my_target, waypoints, formation_changed);
            return;
        }
    }

    // Run Hungarian algorithm to find optimal assignment
    std::vector<int> assignment = hungarianAssignment(current_positions, target_positions);

    // Find my assigned target
    Eigen::Vector3d my_target = target_positions[assignment[drone_id_]];

    RCLCPP_INFO(node_->get_logger(),
               "Drone %d assigned to target %d: (%.2f, %.2f, %.2f) via Hungarian algorithm",
               drone_id_, assignment[drone_id_], my_target.x(), my_target.y(), my_target.z());

    // Publish formation target
    publishFormationTarget(my_target, waypoints, formation_changed);
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
        pattern.push_back(Eigen::Vector3d( -scale/2,  -scale/2, 0.0));
        pattern.push_back(Eigen::Vector3d( -scale/2,  scale/2, 0.0));
        pattern.push_back(Eigen::Vector3d( scale/2, scale/2, 0.0));
        pattern.push_back(Eigen::Vector3d( scale/2, -scale/2, 0.0));
    }
    else if (formation_type == "triangle" && num_drones >= 3) {
        double h = scale * std::sqrt(3) / 2.0;

        pattern.clear();
        // Original triangle: apex points up
        pattern.push_back(Eigen::Vector3d(0.0, -2.0*h/3.0, 0.0));         // Bottom center (apex)
        pattern.push_back(Eigen::Vector3d(-scale/2.0, +h/3.0, 0.0));      // Top left
        pattern.push_back(Eigen::Vector3d(0.0, +h/3.0, 0.0));             // Top center

        if (num_drones > 3) {
            pattern.push_back(Eigen::Vector3d(+scale/2.0, +h/3.0, 0.0));  // Top right
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

void ReplanFSM::publishFormationTarget(const Eigen::Vector3d& target, const std::vector<Eigen::Vector3d>& waypoints, bool formation_changed) {
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
    const std::vector<Eigen::Vector3d>& target_positions)
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

    // Compute formation centers and orientations
    Eigen::Vector3d current_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        current_center += current_positions[i];
        target_center += target_positions[i];
    }
    current_center /= n;
    target_center /= n;

    // Initial cost computation
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            // 1. Basic distance cost
            double distance = (current_positions[i] - target_positions[j]).norm();

            // 2. Orientation-aware cost (preserve relative formation position)
            Eigen::Vector3d current_relative = current_positions[i] - current_center;
            Eigen::Vector3d target_relative = target_positions[j] - target_center;

            // Cost is lower if relative positions are similar
            // This helps maintain formation structure during transitions
            double orientation_error = (current_relative - target_relative).norm();

            // 3. Path crossing penalty (rough estimate)
            double crossing_penalty = 0.0;
            for (int k = 0; k < n; ++k) {
                if (k == i) continue;

                // Check if paths might cross
                // Path i: current_positions[i] → target_positions[j]
                // Path k: current_positions[k] → target_positions[k] (assume identity for initial estimate)
                Eigen::Vector3d path_i = target_positions[j] - current_positions[i];
                Eigen::Vector3d path_k = target_positions[k] - current_positions[k];

                // If paths go in opposite directions from center, likely to cross
                if (path_i.dot(path_k) < 0) {
                    crossing_penalty += 1.0;
                }
            }

            // Combined cost function
            cost_matrix[i][j] =
                1.0 * distance +              // Base distance (weight: 1.0)
                1.5 * orientation_error +     // Formation structure preservation (weight: 1.5)
                0.8 * crossing_penalty;       // Crossing avoidance (weight: 0.8)
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
               "[HUNGARIAN] Cost matrix computed with orientation-aware and crossing penalties");
    log_manager_->infof("[HUNGARIAN] Cost matrix computed with orientation-aware and crossing penalties");

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
    std::vector<int> assignment(n);
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
                // Fallback to identity assignment for this drone
                if (drone_idx >= 0 && drone_idx < n) {
                    assignment[drone_idx] = drone_idx;
                }
            }
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
