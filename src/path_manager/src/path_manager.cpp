#include "path_manager/path_manager.h"

namespace path_manager
{

    PathManager::PathManager(rclcpp::Node::SharedPtr node)
        : node_(node),
          max_vel_(-1.0),
          max_acc_(-1.0),
          poly_traj_piece_length_(-1.0),
          planning_horizen_(-1.0),
          is_optimizer_initialized_(false),
          first_call_(true)
    {
        int drone_id;
        node_->get_parameter("drone_id", drone_id);
        traj_.local_traj.drone_id = drone_id;

        node_->declare_parameter("manager/max_vel", -1.0);
        node_->declare_parameter("manager/max_acc", -1.0);
        node_->declare_parameter("manager/polyTraj_piece_length", -1.0);
        node_->declare_parameter("manager/planning_horizon", -1.0);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/polyTraj_piece_length", poly_traj_piece_length_);
        node_->get_parameter("manager/planning_horizon", planning_horizen_);

        grid_map_ = std::make_shared<GridMap>();
        grid_map_->initMap(node_);

        Eigen::Vector3i voxel_num = grid_map_->getVoxelNum();
        int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
        std::vector<double> static_map(buffer_size, 0.0);
        grid_map_->setStaticMap(static_map);

        node_->declare_parameter("obstacles", std::vector<double>{});
        std::vector<double> obstacle_params;
        node_->get_parameter("obstacles", obstacle_params);

        for (size_t i = 0; i < obstacle_params.size(); i += 3)
        {
            obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], obstacle_params[i + 2]);
        }

        std::cout << "Obstacle centers: ";
        for (const auto &obs : obstacle_centers_)
        {
            std::cout << "(" << obs.x() << ", " << obs.y() << ", " << obs.z() << ") ";
        }
        std::cout << std::endl;

        for (const auto &obs : obstacle_centers_)
        {
            Eigen::Vector3i idx;
            grid_map_->posToIndex(obs, idx);
            grid_map_->setOccupancy(idx, 1.0);
            grid_map_->inflatePoint(idx, 3.0);
        }
        grid_map_->updateESDF3d(); // not used? -> esdf_timer

        simple_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/drone_" + std::to_string(drone_id) + "/simple_path", 10);
    }

    void PathManager::updateRobotState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& local_target_pt)
    {
        current_start_pt_ = start_pt;
        current_target_pt_ = local_target_pt;
        has_valid_state_ = true;
    }

    void PathManager::initOptimizer()
    {
        if (is_optimizer_initialized_ && poly_traj_opt_)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Optimizer already initialized for drone %d", traj_.local_traj.drone_id);
            return;
        }
        
        // Reset state in case of partial initialization
        is_optimizer_initialized_ = false;
        poly_traj_opt_.reset();
        
        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing optimizer for drone %d...", traj_.local_traj.drone_id);
            
            // Check prerequisites
            if (!node_) {
                throw std::runtime_error("Node is null");
            }
            if (!grid_map_) {
                throw std::runtime_error("GridMap is null");
            }
            
            poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
            
            // Set parameters first to ensure node_ is initialized
            poly_traj_opt_->setParam(node_);
            
            // Then set other components
            poly_traj_opt_->setEnvironment(grid_map_);
            poly_traj_opt_->setDroneId(traj_.local_traj.drone_id);

            // Only mark as initialized after all steps succeed
            is_optimizer_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Optimizer initialized successfully for drone %d", traj_.local_traj.drone_id);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during optimizer initialization: %s", e.what());
            poly_traj_opt_.reset();  // Reset to nullptr on failure
            is_optimizer_initialized_ = false;
            throw;  // Re-throw the exception
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown exception during optimizer initialization");
            poly_traj_opt_.reset();
            is_optimizer_initialized_ = false;
            throw;
        }
    }

    bool PathManager::computeAndOptimizePath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const double trajectory_start_time, const Eigen::Vector3d &local_target_pt,
                                             const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
                                             const bool flag_randomPolyTraj, const bool use_formation, const bool have_local_traj)
    {
        static int count = 0;
        RCLCPP_INFO(node_->get_logger(), 
                   "\033[47;30m\n[drone %d replan %d]==============================================\033[0m",
                   traj_.local_traj.drone_id, count++);

        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            RCLCPP_INFO(node_->get_logger(), "Close to goal");
            return false;
        }
        auto t_start = rclcpp::Clock(RCL_ROS_TIME).now();
        
        /*** STEP 1: INIT ***/
        double ts = poly_traj_piece_length_ / max_vel_;
        poly_traj::MinJerkOpt initMJO;
        if (!computeInitReferenceState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, ts, initMJO, flag_polyInit))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to compute initial reference state.");
            return false;
        }

        auto t_init = rclcpp::Clock(RCL_ROS_TIME).now() - t_start;

        Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(poly_traj_opt_->get_cps_num_prePiece_());
        poly_traj_opt_->setControlPoints(cstr_pts);

        t_start = rclcpp::Clock(RCL_ROS_TIME).now();

        /*** STEP 2: OPTIMIZE ***/
        poly_traj::Trajectory initTraj = initMJO.getTraj();
        int PN = initTraj.getPieceNum();
        Eigen::MatrixXd all_pos = initTraj.getPositions();
        Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
        tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);

        bool flag_success = poly_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState, innerPts, initTraj.getDurations(), cstr_pts, use_formation);
        
        auto t_opt = rclcpp::Clock(RCL_ROS_TIME).now() - t_start;
        
        if (!flag_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to optimize trajectory.");
            return false;
        }

        // Performance statistics
        static double sum_time = 0;
        static int count_success = 0;
        double total_time_sec = (t_init.nanoseconds() + t_opt.nanoseconds()) / 1e9;
        sum_time += total_time_sec;
        count_success++;
        
        RCLCPP_INFO(node_->get_logger(), 
                   "total time:\033[42m%.3f\033[0m,init:%.3f,optimize:%.3f,avg_time=%.3f,count_success=%d",
                   total_time_sec, t_init.nanoseconds() / 1e9, t_opt.nanoseconds() / 1e9, 
                   sum_time / count_success, count_success);

        if (have_local_traj && use_formation)
        {
            double delta_replan_time = trajectory_start_time - rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            if (delta_replan_time > 0)
            {
                // RCLCPP_INFO(node_->get_logger(), "Waiting for %.2f seconds to sync start time", delta_replan_time);
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(delta_replan_time)));
            }
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
        }
        else
        {
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(),
                            rclcpp::Clock(RCL_ROS_TIME).now().seconds());
        }
        return true;
    }

    bool PathManager::computeInitReferenceState(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                                const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
                                                const Eigen::Vector3d &local_target_vel, const double &ts,
                                                poly_traj::MinJerkOpt &initMJO, const bool flag_polyInit)
    {
        if (first_call_ || flag_polyInit) {
        first_call_ = false;
        Eigen::Matrix3d headState, tailState;
        headState << start_pt, start_vel, start_acc;
        tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

        Eigen::MatrixXd ctl_points;

        auto t1 = rclcpp::Clock(RCL_ROS_TIME).now();
        poly_traj_opt_->astarWithMinTraj(headState, tailState, simple_path_, ctl_points, initMJO);

        auto t2 = rclcpp::Clock(RCL_ROS_TIME).now();
        double duration_ms = (t2 - t1).nanoseconds() / 1e6;

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        path_msg.header.frame_id = "world";

        for (const auto &point : simple_path_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        simple_path_pub_->publish(path_msg);
    }
        else
        {
            if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
            {
                return false;
            }

            double passed_t_on_lctraj = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - traj_.local_traj.start_time;
            double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
            double t_to_lc_tgt = t_to_lc_end + (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);\

            int piece_nums = std::ceil((start_pt - local_target_pt).norm() / poly_traj_piece_length_);
            if (piece_nums < 2)
            {
                piece_nums = 2;
            }

            Eigen::Matrix3d headState, tailState;
            Eigen::MatrixXd innerPs(3, piece_nums - 1);
            Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
            headState << start_pt, start_vel, start_acc;
            tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

            double t = piece_dur_vec(0);
            for (int i = 0; i < piece_nums - 1; ++i)
            {
                if (t < t_to_lc_end)
                {
                    innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
                }
                else if (t <= t_to_lc_tgt)
                {
                    double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
                    innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Should not happen! x_x 0x88");
                }

                t += piece_dur_vec(i + 1);
            }

            initMJO.reset(headState, tailState, piece_nums);
            initMJO.generate(innerPs, piece_dur_vec);
        }

        return true;
    }

    bool PathManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                                     const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
        poly_traj::MinJerkOpt globalMJO;
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        tailState << waypoints.back(), end_vel, end_acc;
        Eigen::MatrixXd innerPts;

        if (waypoints.size() > 1)
        {
            innerPts.resize(3, waypoints.size() - 1);
            for (int i = 0; i < waypoints.size() - 1; i++)
                innerPts.col(i) = waypoints[i];
        }
        else
        {
            if (innerPts.size() != 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "innerPts.size() != 0");
            }
        }
        globalMJO.reset(headState, tailState, waypoints.size());

        double des_vel = max_vel_;
        Eigen::VectorXd time_vec(waypoints.size());
        int try_num = 0;
        do
        {
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                                       : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
            }
            globalMJO.generate(innerPts, time_vec);
            // cout << "try_num : " << try_num << endl;
            // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
            // cout << "time_vec : " << time_vec.transpose() << endl;

            des_vel /= 1.2;
            try_num++;
        } while (globalMJO.getTraj().getMaxVelRate() > max_vel_ && try_num <= 5);

        auto time_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(globalMJO.getTraj(), time_now);

        return true;
    }

    void PathManager::getLocalTarget(const Eigen::Vector3d &start_pt,
                                     const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                                     Eigen::Vector3d &local_target_vel, double &t_to_target)
    {
        double t;

        traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

        double t_step = planning_horizen_ / 20 / max_vel_;
        // double dist_min = 9999, dist_min_t = 0.0;
        for (t = traj_.global_traj.glb_t_of_lc_tgt;
             t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
             t += t_step)
        {
            Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
            double dist = (pos_t - start_pt).norm();

            if (dist >= planning_horizen_)
            {
                local_target_pos = pos_t;
                traj_.global_traj.glb_t_of_lc_tgt = t;
                break;
            }
        }

        if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration) // Last global point
        {
            local_target_pos = global_end_pt;
            traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
        }

        if ((global_end_pt - local_target_pos).norm() < (max_vel_ * max_vel_) / (2 * max_acc_))
        {
            local_target_vel = Eigen::Vector3d::Zero();
        }
        else
        {
            local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
        }
    }

bool PathManager::checkCollision(int drone_id)
{
    // 기본 유효성 검사
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
        return false;
    
    // 드론 ID 유효성 검사
    if (drone_id < 0 || static_cast<size_t>(drone_id) >= traj_.swarm_traj.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid drone_id %d in checkCollision (swarm_traj size: %zu)", 
                   drone_id, traj_.swarm_traj.size());
        return false;
    }

    // 궤적 유효성 검사
    if (!traj_.local_traj.traj.getPieceNum() || !traj_.swarm_traj[drone_id].traj.getPieceNum()) {
        RCLCPP_WARN(node_->get_logger(), "Empty trajectory in checkCollision for drone_id %d", drone_id);
        return false;
    }

    try {
        double my_traj_start_time = traj_.local_traj.start_time;
        double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;
        
        // 시간 범위 계산
        double t_start = std::max(my_traj_start_time, other_traj_start_time);
        double t_end = std::min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                           other_traj_start_time + traj_.swarm_traj[drone_id].duration);
        
        // 유효한 시간 범위인지 확인
        if (t_start >= t_end) {
            return false;  // 겹치는 시간 구간 없음
        }

        // 메모리 사용량 확인
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        if (usage.ru_maxrss > 7000000) { // 7GB 제한
            RCLCPP_WARN(node_->get_logger(), "High memory usage during collision check: %zu MB", 
                      usage.ru_maxrss / 1024);
        }

        // 충돌 검사 - 시간 간격 증가 (0.03 -> 0.05)
        for (double t = t_start; t < t_end; t += 0.05)
        {
            double my_t = t - my_traj_start_time;
            double other_t = t - other_traj_start_time;
            
            // 시간 범위 유효성 검사
            if (my_t < 0 || my_t > traj_.local_traj.duration || 
                other_t < 0 || other_t > traj_.swarm_traj[drone_id].duration) {
                continue;
            }
            
            // 충돌 검사
            if ((traj_.local_traj.traj.getPos(my_t) -
                traj_.swarm_traj[drone_id].traj.getPos(other_t))
                    .norm() < poly_traj_opt_->getSwarmClearance())
            {
                return true;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in checkCollision: %s", e.what());
        return false;
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Unknown exception in checkCollision");
        return false;
    }

    return false;
}

} // namespace path_manager
