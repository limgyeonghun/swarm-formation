#include "path_manager/replan_fsm.h"
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
      current_time_(0.0),
      last_start_time_(0.0),
      n_seconds_ahead_(0.0), 
      rviz_simulation_ (false) 
    {
    node_->declare_parameter("drone_id", 0);
    node_->get_parameter("drone_id", drone_id_);
    RCLCPP_INFO(node_->get_logger(), "Starting ReplanFSM for drone_id: %d", drone_id_);

    node_->declare_parameter("rviz_simulation", false);
    node_->get_parameter("rviz_simulation", rviz_simulation_);
    RCLCPP_INFO(node_->get_logger(), "rviz_simulation: %s", rviz_simulation_ ? "true" : "false");

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
    node_->declare_parameter("end_point_x", 12.0);
    node_->declare_parameter("end_point_y", 12.0);
    node_->declare_parameter("end_point_z", 0.0);

    double start_x, start_y, start_z, end_x, end_y, end_z;
    node_->get_parameter("start_point_x", start_x);
    node_->get_parameter("start_point_y", start_y);
    node_->get_parameter("start_point_z", start_z);
    node_->get_parameter("end_point_x", end_x);
    node_->get_parameter("end_point_y", end_y);
    node_->get_parameter("end_point_z", end_z);

    std::cout << "ReplanFSM parameters: " << std::endl;
    std::cout << "  start_point_x: " << start_x << std::endl;
    std::cout << "  start_point_y: " << start_y << std::endl;
    std::cout << "  start_point_z: " << start_z << std::endl;
    std::cout << "  end_point_x: " << end_x << std::endl;
    std::cout << "  end_point_y: " << end_y << std::endl;
    std::cout << "  end_point_z: " << end_z << std::endl;

    offset_pt_ = Eigen::Vector3d(start_x, start_y, start_z);
    start_pt_ = offset_pt_;
    end_pt_ = Eigen::Vector3d(end_x, end_y, end_z);
    have_target_ = true;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odom_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string odom_topic = "/vehicle" + std::to_string(drone_id_+1) + "/target_position";
    std::string topic_prefix = "/V" + std::to_string(drone_id_+1);    

    optimized_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>("planning/trajectory", sensor_qos);
    global_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>("planning/global", sensor_qos);
    broadcast_traj_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(topic_prefix + "/planning/broadcast_traj_send", sensor_qos);
    // odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, sensor_qos);

    if (rviz_simulation_)
    {
        std::string position_topic = "/drone_" + std::to_string(drone_id_) + "/current_position";
        position_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            position_topic, sensor_qos, std::bind(&ReplanFSM::positionCallback, this, std::placeholders::_1));
    }
    else
    {
        std::string px4_position_topic = "/vehicle" + std::to_string(drone_id_ + 1) + "/fmu/out/vehicle_local_position";
        px4_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            px4_position_topic, sensor_qos, std::bind(&ReplanFSM::PX4positionCallback, this, std::placeholders::_1));
    }

    broadcast_traj_sub_ = node_->create_subscription<path_manager::msg::PolyTraj>(
        topic_prefix + "/j_fi/broadcast_traj_recv", sensor_qos,
        std::bind(&ReplanFSM::recvBroadcastPolyTrajCallback, this, std::placeholders::_1));

    // odom_timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::publishOdometry, this), odom_callback_group_);
    timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this), timer_callback_group_);

    // odom_timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::publishOdometry, this));
    // timer_ = node_->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this));
}

void ReplanFSM::init()
{
    path_manager_ = std::make_shared<PathManager>(node_);
    path_manager_->initOptimizer();
    path_manager_->deliverTrajToOptimizer();

    Eigen::MatrixXd iniState = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd finState = Eigen::MatrixXd::Zero(3, 3);
    iniState.col(0) = start_pt_;
    finState.col(0) = end_pt_;

    bool success = path_manager_->planGlobalTraj(start_pt_, iniState.col(1), iniState.col(2),
                                                 {end_pt_}, finState.col(1), finState.col(2));
    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Success to generate global trajectory!!!");
        // end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        if (exec_state_ == WAIT_POSITION)
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to generate global trajectory!!!");
    }
    // RCLCPP_INFO(node_->get_logger(), "Generated global trajectory successfully!!!");
    // path_manager::msg::PolyTraj msg;
    // globalTraj2ROSMsg(msg);
    // global_path_pub_->publish(msg);
}

// void ReplanFSM::publishOdometry() {
//     if (exec_state_ != FSM_EXEC_STATE::EXEC_TRAJ || !have_local_traj_) {
//         return;
//     }

//     auto local_traj = &path_manager_->traj_.local_traj;
//     double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - local_traj->start_time;
//     t_cur = std::min(local_traj->duration, t_cur);

//     double t_ahead = std::min(t_cur + n_seconds_ahead_, local_traj->duration);
//     Eigen::Vector3d pos = local_traj->traj.getPos(t_ahead);
//     Eigen::Vector3d vel = local_traj->traj.getVel(t_ahead);

//     // RCLCPP_INFO(node_->get_logger(), "[vehicle %d] t_cur: %f ",drone_id_ + 1, t_cur);

//     nav_msgs::msg::Odometry msg{};
//     msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
//     msg.header.frame_id = "odom";
//     msg.child_frame_id = "base_link";

//     msg.pose.pose.position.x = pos(0);
//     msg.pose.pose.position.y = pos(1);
//     msg.pose.pose.position.z = pos(2);

//     msg.twist.twist.linear.x = vel(0);
//     msg.twist.twist.linear.y = vel(1);
//     msg.twist.twist.linear.z = vel(2);

//     odom_pub_->publish(msg);
// }

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
                bool success = planFromGlobalTraj(1);
                
                if (success)
                {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } 
                else 
                {
                    RCLCPP_ERROR(node_->get_logger(), "MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
                    changeFSMExecState(SEQUENTIAL_START, "FSM");
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
            auto local_traj = &path_manager_->traj_.local_traj;
            double t_cur = current_time_ - local_traj->start_time;
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
                    return;
                }
                else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "No Replan Thresh");
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
            break;
        }
    }
}

void ReplanFSM::positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_pos_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    // RCLCPP_ERROR(node_->get_logger(), "Current position: %.2f, %.2f, %.2f", current_pos_(0), current_pos_(1), current_pos_(2));
    current_time_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    have_position_ = true;
}

void ReplanFSM::PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_pos_(0) = msg->x + offset_pt_(0);
    current_pos_(1) = msg->y + offset_pt_(1);
    current_pos_(2) = offset_pt_(2);

    current_time_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    have_position_ = true;
}

void ReplanFSM::recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    if (msg->drone_id < 0) {
        RCLCPP_ERROR(node_->get_logger(), "drone_id < 0 is not allowed in a swarm system!");
        return;
    }
    if (msg->order != 5) {
        RCLCPP_ERROR(node_->get_logger(), "Only support trajectory order equals 5 now!");
        return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
        RCLCPP_ERROR(node_->get_logger(), "WRONG trajectory parameters.");
        return;
    }
    rclcpp::Time msg_time(msg->start_time);
    double time_diff = (rclcpp::Clock(RCL_ROS_TIME).now() - msg_time).seconds();
    if (std::abs(time_diff) > 0.5) {  // Increased tolerance for embedded systems (0.25 -> 0.5)
        RCLCPP_WARN(node_->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs (rejected)",
                    msg->drone_id, time_diff);
        return;
    } else if (std::abs(time_diff) > 0.25) {
        RCLCPP_WARN(node_->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs (accepted with warning)",
                    msg->drone_id, time_diff);
    }

    const size_t recv_id = static_cast<size_t>(msg->drone_id);
    if (static_cast<int>(recv_id) == drone_id_) {
        return;
    }

    // Ensure swarm_traj vector is large enough and properly initialized
    if (path_manager_->traj_.swarm_traj.size() <= recv_id) {
        for (size_t i = path_manager_->traj_.swarm_traj.size(); i <= recv_id; i++) {
            LocalTrajData blank;
            blank.drone_id = -1;
            blank.traj_id = -1;
            blank.duration = 0.0;
            blank.start_time = 0.0;
            blank.end_time = 0.0;
            blank.start_pos = Eigen::Vector3d::Zero();
            path_manager_->traj_.swarm_traj.push_back(blank);
        }
        RCLCPP_INFO(node_->get_logger(), "Expanded swarm_traj vector to size %zu for drone %d", 
                    path_manager_->traj_.swarm_traj.size(), msg->drone_id);
    }

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

    // RCLCPP_INFO(node_->get_logger(), "Received trajectory from drone %zu, traj_id: %d, duration: %.2f",
    //             recv_id, msg->traj_id, path_manager_->traj_.swarm_traj[recv_id].duration);

    if (!have_recv_pre_agent_ && static_cast<int>(path_manager_->traj_.swarm_traj.size()) >= drone_id_) {
        for (int i = 0; i < drone_id_; ++i) {
            if (path_manager_->traj_.swarm_traj[i].drone_id != i) {
                break;
            }
            have_recv_pre_agent_ = true;
        }
    }
}

void ReplanFSM::polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg) 
{
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

    // if (have_local_traj_) {
    //     desired_start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds() + replan_trajectory_time_;
    //     double t_adj = desired_start_time - path_manager_->traj_.local_traj.start_time;
    //     double t_ahead = std::min(t_adj + n_seconds_ahead_, path_manager_->traj_.local_traj.duration);
        
    //     desired_start_pt = path_manager_->traj_.local_traj.traj.getPos(t_adj);
    //     desired_start_vel = path_manager_->traj_.local_traj.traj.getVel(t_adj);
    //     desired_start_acc = path_manager_->traj_.local_traj.traj.getAcc(t_adj);
    // } else {
    //     desired_start_pt = start_pt_;
    //     desired_start_vel = Eigen::Vector3d(1.0, 0.0, 0.0);
    //     desired_start_acc = Eigen::Vector3d::Zero();
    // }

    // RCLCPP_ERROR(node_->get_logger(), "desired_start_pt: %.2f, %.2f, %.2f", desired_start_pt(0), desired_start_pt(1), desired_start_pt(2));

    bool plan_success = path_manager_->computeAndOptimizePath(
        desired_start_pt, desired_start_vel, desired_start_acc, desired_start_time,
        local_target_pt_, local_target_vel_, flag_use_poly_init, flag_randomPolyTraj,
        use_formation, have_local_traj_);

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
    // start_vel_ = current_vel_; //todo
    start_vel_.setZero();
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++) {
        if (callPathManager(true, false, true)) {
            return true;
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
    exec_state_ = new_state;
}

}  // namespace path_manager
