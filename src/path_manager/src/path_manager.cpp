#include "path_manager/path_manager.h"
#include "path_manager/uniform_bspline.h"
#include "path_manager/polynomial_traj.h"

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
        log_manager_ = std::make_shared<swarm_formation::LogManager>(
            node_->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);
        
        enable_debug_logs_ = false;
        if (node_->has_parameter("enable_debug_logs")) {
            node_->get_parameter("enable_debug_logs", enable_debug_logs_);
        }
            
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
            grid_map_->inflatePoint(idx, 10.0);
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

            // Set LogManager for unified logging
            poly_traj_opt_->setLogManager(log_manager_);
            
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
        // Ensure optimizer is initialized before proceeding
        if (!isOptimizerInitialized()) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot compute trajectory: optimizer not initialized!");
            return false;
        }
        
        static int count = 0;
        if (enable_debug_logs_) {
            log_manager_->infof("=== DRONE %d REPLAN %d START ===", 
                               traj_.local_traj.drone_id, count++);
            log_manager_->infof("Start: (%.2f,%.2f,%.2f) -> Target: (%.2f,%.2f,%.2f), Distance: %.2fm",
                               start_pt(0), start_pt(1), start_pt(2), 
                               local_target_pt(0), local_target_pt(1), local_target_pt(2),
                               (start_pt - local_target_pt).norm());
        }

        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            if (enable_debug_logs_) {
                log_manager_->info("Close to goal");
            }
            return false;
        }
        auto t_start = rclcpp::Clock(RCL_ROS_TIME).now();
        
        /*** STEP 1: INIT ***/
        double ts = poly_traj_piece_length_ / max_vel_;
        poly_traj::MinJerkOpt initMJO;
        if (!computeInitReferenceState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, ts, initMJO, flag_polyInit))
        {
            log_manager_->error("Failed to compute initial reference state.");
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
            log_manager_->error("Failed to optimize trajectory.");
            return false;
        }

        // Performance statistics
        static double sum_time = 0;
        static int count_success = 0;
        double total_time_sec = (t_init.nanoseconds() + t_opt.nanoseconds()) / 1e9;
        sum_time += total_time_sec;
        count_success++;
        
        if (enable_debug_logs_) {
            log_manager_->infof("PERFORMANCE - Total:%.3fms, Init:%.3fms, Optimize:%.3fms, Avg:%.3fms, Success:%d",
                               total_time_sec * 1000, t_init.nanoseconds() / 1e6, t_opt.nanoseconds() / 1e6, 
                               (sum_time / count_success) * 1000, count_success);
        }

        if (have_local_traj && use_formation)
        {
            double delta_replan_time = trajectory_start_time - rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            if (delta_replan_time > 0)
            {
                // RCLCPP_INFO(node_->get_logger(), "Waiting for %.2f seconds to sync start time", delta_replan_time);
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(delta_replan_time)));
            }
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time, traj_.local_traj.drone_id);
        }
        else
        {
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), rclcpp::Clock(RCL_ROS_TIME).now().seconds(), traj_.local_traj.drone_id);
        }
        
        if (enable_debug_logs_) {
            log_manager_->infof("=== DRONE %d REPLAN COMPLETED SUCCESSFULLY ===", 
                               traj_.local_traj.drone_id);
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

    std::vector<Eigen::VectorXd> PathManager::playground_bspline(const std::vector<Eigen::VectorXd> &pts)
    {
        std::vector<Eigen::VectorXd> b_pts;

        double interval = 1.0;
        int num_samples = 300;

        Eigen::MatrixXd ctl_pts(pts[0].size(), pts.size());
        for (size_t i = 0; i < pts.size(); ++i)
        {
            ctl_pts.col(i) = pts[i];
        }

        // B Spline
        ego_planner::UniformBspline bspline = ego_planner::UniformBspline(ctl_pts, 3, interval);
        double ts, te;
        bspline.getTimeSpan(ts, te);
        double gap = (te - ts) / num_samples;
        Eigen::VectorXd pos;
        for (double t = ts; t < te; t += gap)
        {
            pos = bspline.evaluateDeBoorT(t);
            b_pts.push_back(pos);
        }

        return b_pts;
    }

    bool PathManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                                     const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning global trajectory using playground B-spline with %zu waypoints", waypoints.size());

        // Step 1: Prepare waypoints for B-spline generation
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : waypoints) {
            all_points.push_back(wp);
        }

        // Step 2: Convert Vector3d to VectorXd for playground_bspline
        std::vector<Eigen::VectorXd> pts_vectorxd;
        if (!all_points.empty()) {
            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points.front().x(), all_points.front().y(), all_points.front().z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
            for (size_t i = 1; i < all_points.size() - 1; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points[i].x(), all_points[i].y(), all_points[i].z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points.back().x(), all_points.back().y(), all_points.back().z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
        }

        // Step 3: Generate B-spline trajectory using playground_bspline
        std::vector<Eigen::VectorXd> b_pts = playground_bspline(pts_vectorxd);
        
        RCLCPP_INFO(node_->get_logger(), "Generated %zu B-spline points", b_pts.size());

        // Step 4: Convert back to Vector3d for trajectory generation
        std::vector<Eigen::Vector3d> sampled_points;
        for (const auto& b_pt : b_pts) {
            sampled_points.push_back(Eigen::Vector3d(b_pt.x(), b_pt.y(), b_pt.z()));
        }

        // Step 5: Convert to MINCO trajectory
        poly_traj::MinJerkOpt globalMJO;
        
        // Create waypoint trajectory using the sampled points
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        tailState << waypoints.back(), end_vel, end_acc;
        
        Eigen::MatrixXd innerPts;
        if (sampled_points.size() > 2) {
            innerPts.resize(3, sampled_points.size() - 2);
            for (size_t i = 1; i < sampled_points.size() - 1; ++i) {
                innerPts.col(i-1) = sampled_points[i];
            }
        }
        
        globalMJO.reset(headState, tailState, sampled_points.size() - 1);
        
        // Optimize time allocation to ensure velocity constraints
        double des_vel = max_vel_;
        Eigen::VectorXd optimized_time_vec(sampled_points.size() - 1);
        int try_num = 0;
        
        do {
            for (size_t i = 0; i < sampled_points.size() - 1; ++i) {
                double segment_length = (sampled_points[i+1] - sampled_points[i]).norm();
                optimized_time_vec(i) = std::max(0.1, segment_length / des_vel);
            }
            
            globalMJO.generate(innerPts, optimized_time_vec);
            
            des_vel /= 1.2;
            try_num++;
        } while (globalMJO.getTraj().getMaxVelRate() > max_vel_ && try_num <= 5);
        
        auto time_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(globalMJO.getTraj(), time_now);
        
        RCLCPP_INFO(node_->get_logger(), "Successfully generated global trajectory with B-spline -> MINCO conversion");
        RCLCPP_INFO(node_->get_logger(), "Final trajectory: %d segments, duration: %.3f, max_vel: %.3f", 
                   globalMJO.getTraj().getPieceNum(), globalMJO.getTraj().getTotalDuration(), 
                   globalMJO.getTraj().getMaxVelRate());
        
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
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < poly_traj_opt_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
}

bool PathManager::isMapReady(const Eigen::Vector3d& start_pos) const {
    if (!grid_map_) {
        return false;
    }

    if (!grid_map_->isInMap(start_pos)) {
        return false;
    }

    Eigen::Vector3i start_idx;
    grid_map_->posToIndex(start_pos, start_idx);

    int check_radius = 1;
    for (int dx = -check_radius; dx <= check_radius; dx++) {
        for (int dy = -check_radius; dy <= check_radius; dy++) {
            Eigen::Vector3i check_idx = start_idx + Eigen::Vector3i(dx, dy, 0);
            if (grid_map_->isInMap(check_idx)) {
                if (grid_map_->isUnknown(check_idx)) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

} // namespace path_manager
