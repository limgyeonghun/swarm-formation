#include "path_manager/replan_fsm.h"
using namespace std::chrono_literals;

namespace path_manager {

ReplanFSM::ReplanFSM()
    : Node("replan_fsm"),
      exec_state_(FSM_EXEC_STATE::INIT),
      continously_called_times_(0),
      have_position_(true),
      have_target_(false),
      have_new_target_(false),
      have_local_traj_(false),
      have_recv_pre_agent_(false),
      flag_relan_astar_(false),
      drone_id_(0),
      replan_thresh_(-1.0),
      no_replan_thresh_(-1.0),
      replan_trajectory_time_(-1.0),
      current_time_(0.0),
      last_start_time_(0.0) {

    this->declare_parameter("drone_id", 0);
    this->get_parameter("drone_id", drone_id_);
    RCLCPP_INFO(this->get_logger(), "Starting ReplanFSM for drone_id: %d", drone_id_);

    this->declare_parameter("fsm/thresh_replan_time", -1.0);
    this->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    this->declare_parameter("fsm/replan_trajectory_time", -1.0);
    this->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    this->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    this->get_parameter("fsm/replan_trajectory_time", replan_trajectory_time_);

    this->declare_parameter("start_point_x", 0.0);
    this->declare_parameter("start_point_y", 0.0);
    this->declare_parameter("start_point_z", 0.5);
    this->declare_parameter("end_point_x", 12.0);
    this->declare_parameter("end_point_y", 12.0);
    this->declare_parameter("end_point_z", 0.5);

    double start_x, start_y, start_z, end_x, end_y, end_z;
    this->get_parameter("start_point_x", start_x);
    this->get_parameter("start_point_y", start_y);
    this->get_parameter("start_point_z", start_z);
    this->get_parameter("end_point_x", end_x);
    this->get_parameter("end_point_y", end_y);
    this->get_parameter("end_point_z", end_z);

    start_pt_ = Eigen::Vector3d(start_x, start_y, start_z);
    end_pt_ = Eigen::Vector3d(end_x, end_y, end_z);
    have_target_ = true;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

    optimized_path_pub_ = this->create_publisher<path_manager::msg::PolyTraj>("planning/trajectory", sensor_qos);
    global_path_pub_ = this->create_publisher<path_manager::msg::PolyTraj>("planning/global", sensor_qos);
    broadcast_traj_pub_ = this->create_publisher<path_manager::msg::PolyTraj>("planning/broadcast_traj_recv", sensor_qos);

    std::string position_topic = "/drone_" + std::to_string(drone_id_) + "/current_position";
    position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        position_topic, sensor_qos, std::bind(&ReplanFSM::positionCallback, this, std::placeholders::_1));
    broadcast_traj_sub_ = this->create_subscription<path_manager::msg::PolyTraj>(
        "planning/broadcast_traj_recv", sensor_qos,
        std::bind(&ReplanFSM::recvBroadcastPolyTrajCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&ReplanFSM::computeAndPublishPaths, this));
}

void ReplanFSM::init()
{
    path_manager_ = std::make_shared<PathManager>(shared_from_this());
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
        RCLCPP_INFO(this->get_logger(), "Success to generate global trajectory!!!");
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
        RCLCPP_ERROR(this->get_logger(), "Failed to generate global trajectory!!!");
    }
    // RCLCPP_INFO(this->get_logger(), "Generated global trajectory successfully!!!");
    // path_manager::msg::PolyTraj msg;
    // globalTraj2ROSMsg(msg);
    // global_path_pub_->publish(msg);
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
                bool success = planFromGlobalTraj(1);
                
                if (success)
                {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "MY ID :%d have_recv_pre_agent_: %d, Failed to generate the first trajectory!!!", drone_id_,have_recv_pre_agent_);
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
            if (flag_relan_astar_)
                success = planFromLocalTraj(true, false);
            else
                success = planFromLocalTraj(false, true);

            if (success)
            {
                flag_relan_astar_ = false;
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
            else
            {
                flag_relan_astar_ = true;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case EXEC_TRAJ: {
            auto local_traj = &path_manager_->traj_.local_traj;
            double t_cur = current_time_ - local_traj->start_time;
            t_cur = std::min(local_traj->duration, t_cur);

            Eigen::Vector3d pos = local_traj->traj.getPos(t_cur);

            if ((local_target_pt_ - end_pt_).norm() < 0.1) {
                if (t_cur > local_traj->duration - 0.2) {
                    have_target_ = false;
                    have_local_traj_ = false;
                    changeFSMExecState(WAIT_POSITION, "FSM");
                    RCLCPP_INFO(this->get_logger(), "[drone %d reached goal]", drone_id_);
                    return;
                } else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_) {
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
                }
            } else if (t_cur > replan_thresh_) {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case EMERGENCY_STOP: {
            // TODO: Implement emergency stop logic
            break;
        }
    }
}

void ReplanFSM::positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_pos_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    // RCLCPP_ERROR(this->get_logger(), "Current position: %.2f, %.2f, %.2f", current_pos_(0), current_pos_(1), current_pos_(2));
    current_time_ = this->now().seconds();
    have_position_ = true;
}

void ReplanFSM::recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    if (msg->drone_id < 0) {
        RCLCPP_ERROR(this->get_logger(), "drone_id < 0 is not allowed in a swarm system!");
        return;
    }
    if (msg->order != 5) {
        RCLCPP_ERROR(this->get_logger(), "Only support trajectory order equals 5 now!");
        return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
        RCLCPP_ERROR(this->get_logger(), "WRONG trajectory parameters.");
        return;
    }
    rclcpp::Time msg_time(msg->start_time);
    double time_diff = (this->now() - msg_time).seconds();
    if (std::abs(time_diff) > 0.25) {
        RCLCPP_WARN(this->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs",
                    msg->drone_id, time_diff);
        return;
    }

    const size_t recv_id = static_cast<size_t>(msg->drone_id);
    if (static_cast<int>(recv_id) == drone_id_) {
        return;
    }

    if (path_manager_->traj_.swarm_traj.size() <= recv_id) {
        for (size_t i = path_manager_->traj_.swarm_traj.size(); i <= recv_id; i++) {
            LocalTrajData blank;
            blank.drone_id = -1;
            path_manager_->traj_.swarm_traj.push_back(blank);
        }
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

    // RCLCPP_INFO(this->get_logger(), "Received trajectory from drone %zu, traj_id: %d, duration: %.2f",
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
    msg.drone_id = drone_id_;

    auto data = &path_manager_->traj_.local_traj;

    msg.traj_id = data->traj_id;
    rclcpp::Time now = this->now();
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

void ReplanFSM::globalTraj2ROSMsg(path_manager::msg::PolyTraj &msg) 
{
    msg.drone_id = drone_id_;

    auto data = &path_manager_->traj_.global_traj;

    rclcpp::Time now = this->now();
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
        desired_start_time = this->now().seconds() + replan_trajectory_time_;
        double t_adj = desired_start_time - path_manager_->traj_.local_traj.start_time;
        desired_start_pt = path_manager_->traj_.local_traj.traj.getPos(t_adj);
        desired_start_vel = path_manager_->traj_.local_traj.traj.getVel(t_adj);
        desired_start_acc = path_manager_->traj_.local_traj.traj.getAcc(t_adj);
    } else {
        desired_start_pt = start_pt_;
        desired_start_vel = Eigen::Vector3d(1.0, 0.0, 0.0);
        desired_start_acc = Eigen::Vector3d::Zero();
    }

    // RCLCPP_ERROR(this->get_logger(), "desired_start_pt: %.2f, %.2f, %.2f", desired_start_pt(0), desired_start_pt(1), desired_start_pt(2));

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
    double t_debug_start = this->now().seconds();
    LocalTrajData *info = &path_manager_->traj_.local_traj;
    double t_cur = this->now().seconds() - path_manager_->traj_.local_traj.start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

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

    //RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[static_cast<int>(exec_state_)].c_str(), state_str[static_cast<int>(new_state)].c_str());
    exec_state_ = new_state;
}

}  // namespace path_manager