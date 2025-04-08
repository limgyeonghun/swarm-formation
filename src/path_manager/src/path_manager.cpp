#include "path_manager/path_manager.h"
using namespace std::chrono_literals;

PathManager::PathManager()
    : Node("path_manager"),
      is_optimizer_initialized_(false),
      first_call_(true),
      iniState_(3, 3),
      finState_(3, 3),
      last_start_time_(0.0),
      replan_trajectory_time_(0.5) //
{
    // Load obstacle parameters
    this->declare_parameter("obstacles", std::vector<double>{});
    std::vector<double> obstacle_params;
    this->get_parameter("obstacles", obstacle_params);
    if (obstacle_params.empty() || obstacle_params.size() % 3 != 0)
    {
        obstacle_centers_ = {
            Eigen::Vector3d(-2.0, -2.25, 0.5),
            Eigen::Vector3d(1.0, 0.0, 0.5),
            Eigen::Vector3d(0.0, 1.0, 0.5),
            Eigen::Vector3d(3.0, 3.0, 0.5)};
    }
    else
    {
        for (size_t i = 0; i < obstacle_params.size(); i += 3)
        {
            obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], 0.5);
        }
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

    optimized_path_pub_ = this->create_publisher<path_manager::msg::PolyTraj>("optimized_path", sensor_qos);
    position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/current_position", sensor_qos,
        std::bind(&PathManager::positionCallback, this, std::placeholders::_1));

    // Initialize grid map
    Eigen::Vector3d map_size(50.0, 20.0, 5.0);
    double resolution = 0.1;
    grid_map_ = std::make_shared<GridMap>();
    grid_map_->initMap(map_size, resolution);

    Eigen::Vector3i voxel_num = grid_map_->getVoxelNum();
    int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
    std::vector<double> static_map(buffer_size, 0.0);
    grid_map_->setStaticMap(static_map);

    // Process obstacles
    for (const auto &obs : obstacle_centers_)
    {
        Eigen::Vector3i idx;
        grid_map_->posToIndex(obs, idx);
        RCLCPP_INFO(this->get_logger(), "Obstacle at (%.2f, %.2f, %.2f) -> index (%d, %d, %d)",
                    obs.x(), obs.y(), obs.z(), idx.x(), idx.y(), idx.z());
        grid_map_->setOccupancy(idx, 1.0);
        grid_map_->inflatePoint(idx, 5);
    }
    grid_map_->updateESDF3d();

    // Setup trajectory optimizer
    poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
    poly_traj_opt_->setEnvironment(grid_map_);
    poly_traj_opt_->setDroneId(0);

    // Initialize states
    iniState_.col(0) << -4.0, -4.0, 0.5;
    iniState_.col(1) << 1.0, 0.0, 0.0;
    iniState_.col(2) << 0.0, 0.0, 0.0;

    finState_.col(0) << 10.0, 9.5, 0.5;
    finState_.col(1) << 0.0, 0.0, 0.0;
    finState_.col(2) << 0.0, 0.0, 0.0;

    start_pt_ = iniState_.col(0);
    end_pt_ = finState_.col(0);

    timer_ = this->create_wall_timer(500ms, std::bind(&PathManager::computeAndPublishPaths, this));
}

void PathManager::positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    start_pt_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    iniState_.col(0) = start_pt_;
}

void PathManager::initOptimizer()
{
    if (!is_optimizer_initialized_)
    {
        poly_traj_opt_->setParam(shared_from_this());
        is_optimizer_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "PolyTrajOptimizer parameters set.");
    }
}

void PathManager::polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg, const poly_traj::Trajectory &traj)
{
    msg.drone_id = 0;
    msg.traj_id = 1;
    msg.start_time = this->now();
    msg.order = 5;
    int piece_num = traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
        msg.duration[i] = traj.getPiece(i).getDuration();
        poly_traj::CoefficientMat cMat = traj.getPiece(i).getCoeffMat();
        int offset = i * 6;
        for (int j = 0; j < 6; ++j)
        {
            msg.coef_x[offset + j] = cMat(0, j);
            msg.coef_y[offset + j] = cMat(1, j);
            msg.coef_z[offset + j] = cMat(2, j);
        }
    }
}

void PathManager::getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
                                 const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                                 Eigen::Vector3d &local_target_vel, double &t_to_target)
{
    global_traj_.last_glb_t_of_lc_tgt = global_traj_.glb_t_of_lc_tgt;

    double t = global_traj_.glb_t_of_lc_tgt;
    const double t_step = planning_horizen / 20 / max_vel_;
    double max_t = global_traj_.global_start_time + global_traj_.duration;

    while (t < max_t)
    {
        Eigen::Vector3d pos_t = global_traj_.traj.getPos(t - global_traj_.global_start_time);
        double dist = (pos_t - start_pt).norm();
        if (dist >= planning_horizen || (global_end_pt - start_pt).norm() <= planning_horizen)
        {
            local_target_pos = pos_t;
            global_traj_.glb_t_of_lc_tgt = t;
            t_to_target = t - global_traj_.last_glb_t_of_lc_tgt;
            break;
        }
        t += t_step;
    }

    if (t >= max_t || (global_end_pt - start_pt).norm() <= planning_horizen)
    {
        local_target_pos = global_end_pt;
        global_traj_.glb_t_of_lc_tgt = max_t;
        t_to_target = max_t - global_traj_.last_glb_t_of_lc_tgt;
    }

    if ((global_end_pt - local_target_pos).norm() < (max_vel_ * max_vel_) / (2 * max_acc_))
    {
        local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
        local_target_vel = global_traj_.traj.getVel(t - global_traj_.global_start_time);
    }

    double remaining_time = global_traj_.duration - (global_traj_.last_glb_t_of_lc_tgt - global_traj_.global_start_time);
    t_to_target = std::min(t_to_target, remaining_time);
    if (t_to_target < 0.0)
        t_to_target = remaining_time;
}

void PathManager::computeAndPublishPaths()
{
    initOptimizer();
    if ((start_pt_ - end_pt_).norm() < 0.1)
    {
        RCLCPP_INFO(this->get_logger(), "Reached goal! Stopping timer.");
        timer_->cancel();
        return;
    }

    static poly_traj::Trajectory last_optimized_traj;
    const double planning_horizen = 4.0;
    Eigen::Vector3d local_target_pos, local_target_vel;
    double t_to_target;

    if (first_call_)
    {
        std::vector<Eigen::Vector3d> simple_path;
        Eigen::MatrixXd control_points;
        poly_traj::MinJerkOpt init_mjo;
        poly_traj_opt_->astarWithMinTraj(iniState_, finState_, simple_path, control_points, init_mjo);
        if (simple_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "A* failed to find a global path!");
            return;
        }
        simple_path_ = simple_path;
        global_traj_.traj = init_mjo.getTraj();
        global_traj_.global_start_time = this->now().seconds();
        global_traj_.duration = global_traj_.traj.getTotalDuration();
        global_traj_.glb_t_of_lc_tgt = global_traj_.global_start_time;
        global_traj_.last_glb_t_of_lc_tgt = -1.0;
        last_optimized_traj = init_mjo.getTraj();

        local_traj_.drone_id = -1;
        local_traj_.traj_id = 1;
        local_traj_.duration = init_mjo.getTraj().getTotalDuration();
        local_traj_.start_pos = start_pt_;
        local_traj_.start_time = this->now().seconds();
        local_traj_.traj = init_mjo.getTraj();

        path_manager::msg::PolyTraj global_msg;
        polyTraj2ROSMsg(global_msg, global_traj_.traj);
        optimized_path_pub_->publish(global_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial A* global trajectory with %zu pieces, duration: %.2f",
                    global_traj_.traj.getPieceNum(), global_traj_.duration);
        first_call_ = false;
    }

    getLocalTarget(planning_horizen, start_pt_, end_pt_, local_target_pos, local_target_vel, t_to_target);

    if (t_to_target < 0.5)
    {
        RCLCPP_INFO(this->get_logger(), "Adjusting t_to_target from %.2f to 0.5", t_to_target);
        t_to_target = 0.5;
        double t = global_traj_.last_glb_t_of_lc_tgt + t_to_target;
        double max_t = global_traj_.global_start_time + global_traj_.duration;
        if (t < max_t)
        {
            local_target_pos = global_traj_.traj.getPos(t - global_traj_.global_start_time);
            global_traj_.glb_t_of_lc_tgt = t;
            local_target_vel = global_traj_.traj.getVel(t - global_traj_.global_start_time);
        }
        else
        {
            local_target_pos = end_pt_;
            global_traj_.glb_t_of_lc_tgt = max_t;
        }
    }

    double desired_start_time = this->now().seconds() + replan_trajectory_time_;
    Eigen::MatrixXd headState = iniState_;
    if (last_optimized_traj.getPieceNum() > 0)
    {
        double passed_time = desired_start_time - local_traj_.start_time;
        double t_adj = std::min(passed_time, local_traj_.duration);
        headState.col(0) = start_pt_;
        headState.col(1) = last_optimized_traj.getVel(t_adj);
        headState.col(2) = last_optimized_traj.getAcc(t_adj);

        if ((local_traj_.start_pos - start_pt_).norm() > 0.1)
        {
            RCLCPP_WARN(this->get_logger(), "Discontinuity detected: start_pos (%.2f, %.2f, %.2f) vs start_pt (%.2f, %.2f, %.2f)",
                        local_traj_.start_pos(0), local_traj_.start_pos(1), local_traj_.start_pos(2),
                        start_pt_(0), start_pt_(1), start_pt_(2));
            local_traj_.start_pos = start_pt_;
        }
    }

    Eigen::MatrixXd tailState(3, 3);
    tailState.col(0) = local_target_pos;
    tailState.col(1) = local_target_vel;
    tailState.col(2) << 0.0, 0.0, 0.0;

    poly_traj::MinJerkOpt local_mjo;
    double polyTraj_piece_length = 1.0;
    int piece_nums = std::max(2, static_cast<int>(ceil((start_pt_ - local_target_pos).norm() / polyTraj_piece_length)));

    if (t_to_target < 1.0 && piece_nums > 2)
    {
        piece_nums = 2;
        RCLCPP_INFO(this->get_logger(), "Reduced piece_nums to %d due to short t_to_target (%.2f)", piece_nums, t_to_target);
    }

    Eigen::MatrixXd innerPs(3, piece_nums - 1);
    Eigen::VectorXd piece_dur_vec;

    if (global_traj_.last_glb_t_of_lc_tgt < 0.0)
    {
        piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_target / piece_nums);
        double t = 0.0;
        double t_step = t_to_target / piece_nums;
        for (int i = 1; i < piece_nums; ++i)
        {
            t += t_step;
            double traj_t = std::min(t, global_traj_.duration);
            innerPs.col(i - 1) = global_traj_.traj.getPos(traj_t);
            RCLCPP_INFO(this->get_logger(), "Initial Inner Point %d at t=%.2f: (%.2f, %.2f, %.2f)", i - 1, traj_t,
                        innerPs(0, i - 1), innerPs(1, i - 1), innerPs(2, i - 1));
        }
    }
    else
    {
        double passed_t_on_lctraj = this->now().seconds() - local_traj_.start_time;
        double t_to_lc_end = std::max(0.0, local_traj_.duration - passed_t_on_lctraj);
        double t_to_lc_tgt = t_to_lc_end + t_to_target;

        if (t_to_lc_tgt < 0.5)
        {
            RCLCPP_WARN(this->get_logger(), "t_to_lc_tgt adjusted to 0.5 from %.2f", t_to_lc_tgt);
            t_to_lc_tgt = 0.5;
        }

        piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);

        RCLCPP_INFO(this->get_logger(), "passed_t_on_lctraj: %.2f, t_to_lc_end: %.2f, t_to_lc_tgt: %.2f",
                    passed_t_on_lctraj, t_to_lc_end, t_to_lc_tgt);

        double t = 0.0;
        double t_step = t_to_lc_tgt / (piece_nums - 1);
        for (int i = 0; i < piece_nums - 1; ++i)
        {
            t += t_step;
            if (t < t_to_lc_end)
            {
                double t_adj = std::min(t + passed_t_on_lctraj, local_traj_.duration);
                innerPs.col(i) = local_traj_.traj.getPos(t_adj);
            }
            else
            {
                double glb_t = global_traj_.last_glb_t_of_lc_tgt + (t - t_to_lc_end) - global_traj_.global_start_time;
                if (glb_t < 0.0) glb_t = 0.0;
                if (glb_t > global_traj_.duration) glb_t = global_traj_.duration;
                innerPs.col(i) = global_traj_.traj.getPos(glb_t);
            }
            RCLCPP_INFO(this->get_logger(), "Inner Point %d at t=%.2f: (%.2f, %.2f, %.2f)", i, t,
                        innerPs(0, i), innerPs(1, i), innerPs(2, i));
        }
    }

    local_mjo.reset(headState, tailState, piece_nums);
    local_mjo.generate(innerPs, piece_dur_vec);

    int cps_num_prePiece = poly_traj_opt_->get_cps_num_prePiece_();
    Eigen::MatrixXd cstr_pts = local_mjo.getInitConstrainPoints(cps_num_prePiece);
    poly_traj_opt_->setControlPoints(cstr_pts);

    double optimization_start_time = this->now().seconds();
    bool flag_success = poly_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState,
                                                                 innerPs, piece_dur_vec,
                                                                 cstr_pts, false);
    double optimization_time = (this->now().seconds() - optimization_start_time) * 1000.0;

    if (flag_success)
    {
        poly_traj::Trajectory optimized_traj = poly_traj_opt_->getMinJerkOptPtr()->getTraj();
        last_optimized_traj = optimized_traj;

        local_traj_.drone_id = -1;
        local_traj_.traj_id++;
        local_traj_.duration = optimized_traj.getTotalDuration();
        local_traj_.start_pos = start_pt_;
        local_traj_.start_time = optimization_start_time + (optimization_time / 1000.0);
        local_traj_.traj = optimized_traj;

        path_manager::msg::PolyTraj opt_msg;
        polyTraj2ROSMsg(opt_msg, optimized_traj);
        optimized_path_pub_->publish(opt_msg);
        RCLCPP_INFO(this->get_logger(), "Published optimized path with %zu pieces, duration: %.2f, optimization took %.2f ms",
                    optimized_traj.getPieceNum(), optimized_traj.getTotalDuration(), optimization_time);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Optimization failed, keeping previous trajectory.");
        if (last_optimized_traj.getPieceNum() > 0)
        {
            local_traj_.traj = last_optimized_traj;
        }
    }

    double passed_t_on_lctraj = this->now().seconds() - local_traj_.start_time;
    double t_to_lc_end = std::max(0.0, local_traj_.duration - passed_t_on_lctraj);
    double t_to_lc_tgt = t_to_lc_end + t_to_target;
    RCLCPP_INFO(this->get_logger(), "passed_t_on_lctraj: %.2f, t_to_lc_end: %.2f, t_to_lc_tgt: %.2f",
                passed_t_on_lctraj, t_to_lc_end, t_to_lc_tgt);
}