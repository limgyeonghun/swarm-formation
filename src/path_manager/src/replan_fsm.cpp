#include "path_manager/replan_fsm.h"
#include <cmath>
#include <sys/resource.h>
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
      flag_escape_emergency_(false)
    {
        log_manager_ = std::make_unique<swarm_formation::LogManager>(
            node->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);

    node_->declare_parameter("enable_debug_logs", false);
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);
    
    node_->declare_parameter("enable_lbfgs_detail_logs", false);
    // Note: enable_lbfgs_detail_logs will be read by PolyTrajOptimizer::setParam()
    
    node_->declare_parameter("drone_id", 0);
    node_->get_parameter("drone_id", drone_id_);
    FSM_LOG_INFO("Starting ReplanFSM for drone_id: %d", drone_id_);

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
        std::string target_position_topic = "/vehicle" + std::to_string(drone_id_ + 1) + "/target_position";
        target_position_sub_ = node_->create_subscription<path_manager::msg::PositionCommand>(
            target_position_topic, sensor_qos, std::bind(&ReplanFSM::targetPositionCallback, this, std::placeholders::_1));
        have_position_ = true;  // We have initial position from parameters
    }
    else
    {
        std::string px4_position_topic = "/vehicle" + std::to_string(drone_id_ + 1) + "/fmu/out/vehicle_local_position_v1";
        px4_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_position_topic, sensor_qos, std::bind(&ReplanFSM::PX4positionCallback, this, std::placeholders::_1));
    }

    broadcast_traj_sub_ = node_->create_subscription<path_manager::msg::PolyTraj>(
        topic_prefix + "/j_fi/broadcast_traj_recv", sensor_qos,
        std::bind(&ReplanFSM::recvBroadcastPolyTrajCallback, this, std::placeholders::_1));

    formation_target_sub_ = node_->create_subscription<path_manager::msg::FormationTarget>(
        "formation_targets", sensor_qos,
        std::bind(&ReplanFSM::formationTargetCallback, this, std::placeholders::_1));

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
                
                if (success)
                {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } 
                else 
                {
                    RCLCPP_ERROR(node_->get_logger(), "MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    log_manager_->errorf("MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    // Instead of going back to SEQUENTIAL_START immediately, wait a bit
                    static int sequential_start_failures = 0;
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
    // Debug log to show position updates
    RCLCPP_DEBUG(node_->get_logger(), "Updated position from traj_server: (%.2f, %.2f, %.2f)", 
                current_pos_(0), current_pos_(1), current_pos_(2));
}

void ReplanFSM::PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_pos_(0) = msg->x + offset_pt_(0);
    current_pos_(1) = msg->y + offset_pt_(1);
    current_pos_(2) = offset_pt_(2);
    have_position_ = true;
}

void ReplanFSM::recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    if (!path_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "PathManager is not initialized!");
        log_manager_->errorf("PathManager is not initialized!");
        return;
    }
    if (msg->drone_id < 0) {
        RCLCPP_ERROR(node_->get_logger(), "drone_id < 0 is not allowed in a swarm system!");
        log_manager_->errorf("drone_id < 0 is not allowed in a swarm system!");
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

    bool success = false;
    end_pt_ = Eigen::Vector3d(
        msg->target_position.x,
        msg->target_position.y,
        msg->target_position.z);

    success = path_manager_->planGlobalTraj(
        current_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        {end_pt_}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());


    if (success) {
        std::vector<Eigen::Vector3d> formation_positions;
        formation_positions.reserve(msg->formation_positions.size());
            
        for (const auto& pos : msg->formation_positions) {
            formation_positions.emplace_back(pos.x, pos.y, pos.z);
        }
        path_manager_->setFormationToOptimizer(formation_positions, formation_positions.size());

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


}  // namespace path_manager
