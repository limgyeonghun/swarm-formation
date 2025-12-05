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
          first_call_(true),
          current_drone_id_(-1),
          current_formation_type_("")
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
        node_->declare_parameter("manager/intermediate_waypoint_ratio", 0.3);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/polyTraj_piece_length", poly_traj_piece_length_);
        node_->get_parameter("manager/planning_horizon", planning_horizen_);
        node_->get_parameter("manager/intermediate_waypoint_ratio", intermediate_waypoint_ratio_);

        // Clamp intermediate_waypoint_ratio to valid range (0.0, 1.0)
        if (intermediate_waypoint_ratio_ <= 0.0 || intermediate_waypoint_ratio_ >= 1.0) {
            RCLCPP_WARN(node_->get_logger(),
                       "Invalid intermediate_waypoint_ratio: %.2f. Clamping to 0.3",
                       intermediate_waypoint_ratio_);
            intermediate_waypoint_ratio_ = 0.3;
        }

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

        // Obstacle centers logged to file only
        // std::cout << "Obstacle centers: ";
        // for (const auto &obs : obstacle_centers_)
        // {
        //     std::cout << "(" << obs.x() << ", " << obs.y() << ", " << obs.z() << ") ";
        // }
        // std::cout << std::endl;

        // Calculate inflation step from obstacles_inflation parameter
        int inf_step = ceil(grid_map_->getObstaclesInflation() / grid_map_->getResolution());
        
        for (const auto &obs : obstacle_centers_)
        {
            Eigen::Vector3i idx;
            grid_map_->posToIndex(obs, idx);
            grid_map_->setOccupancy(idx, 1.0);
            grid_map_->inflatePoint(idx, inf_step);
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

    void PathManager::initOptimizer(bool force_reinit)
    {
        if (is_optimizer_initialized_ && poly_traj_opt_ && !force_reinit)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Optimizer already initialized for drone %d", traj_.local_traj.drone_id);
            return;
        }

        if (force_reinit && is_optimizer_initialized_) {
            RCLCPP_INFO(node_->get_logger(), "Force reinitializing optimizer for drone %d", traj_.local_traj.drone_id);
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
            if (enable_debug_logs_) {
                log_manager_->infof("[TIMING] delta_replan_time=%.3f ms (trajectory_start_time=%.3f, now=%.3f)",
                                   delta_replan_time * 1000, trajectory_start_time, rclcpp::Clock(RCL_ROS_TIME).now().seconds());
            }
            if (delta_replan_time > 0)
            {
                if (enable_debug_logs_) {
                    log_manager_->infof("[TIMING] Sleeping for %.3f ms to sync trajectory start time", delta_replan_time * 1000);
                }
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(delta_replan_time)));
            }
            auto set_traj_start = std::chrono::high_resolution_clock::now();
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time, traj_.local_traj.drone_id);
            auto set_traj_end = std::chrono::high_resolution_clock::now();
            auto set_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(set_traj_end - set_traj_start).count();
            if (enable_debug_logs_) {
                log_manager_->infof("[TIMING] setLocalTraj took %ld ms", set_traj_duration);
            }
        }
        else
        {
            auto set_traj_start = std::chrono::high_resolution_clock::now();
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), rclcpp::Clock(RCL_ROS_TIME).now().seconds(), traj_.local_traj.drone_id);
            auto set_traj_end = std::chrono::high_resolution_clock::now();
            auto set_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(set_traj_end - set_traj_start).count();
            if (enable_debug_logs_) {
                log_manager_->infof("[TIMING] setLocalTraj took %ld ms", set_traj_duration);
            }
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
        log_manager_->infof("Planning global trajectory using playground B-spline with %zu waypoints", waypoints.size());

        // Safety check: Need at least 1 waypoint for trajectory
        if (waypoints.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "planGlobalTraj: No waypoints provided!");
            return false;
        }

        // Step 1: Use waypoints directly without adjustment (skip outer/inner line calculation)
        // std::vector<Eigen::Vector3d> adjusted_waypoints = adjustWaypointsForFormation(waypoints, start_pos);
        std::vector<Eigen::Vector3d> adjusted_waypoints = waypoints;  // Use original waypoints directly

        // Step 2: Add intermediate alignment waypoint for smoother trajectory
        // The intermediate point is placed between start and first waypoint to ensure
        // the global path is not too straight (nearly a direct line)
        std::vector<Eigen::Vector3d> waypoints_with_intermediate;

        if (!adjusted_waypoints.empty()) {
            // Calculate intermediate alignment point
            // Position it at ratio * distance from start to first waypoint
            Eigen::Vector3d first_waypoint = adjusted_waypoints.front();
            Eigen::Vector3d direction = (first_waypoint - start_pos).normalized();
            double distance_to_first = (first_waypoint - start_pos).norm();

            // Intermediate point: place it at intermediate_waypoint_ratio_ of the way to first waypoint
            // Important: This must be AHEAD of start_pos (not behind)
            Eigen::Vector3d intermediate_point = start_pos + direction * (distance_to_first * intermediate_waypoint_ratio_);

            // Verify intermediate point is not behind start position
            Eigen::Vector3d start_to_intermediate = intermediate_point - start_pos;
            if (start_to_intermediate.dot(direction) > 0.0) {
                // Add intermediate waypoint only if it's ahead of start
                waypoints_with_intermediate.push_back(intermediate_point);

                log_manager_->infof("Added intermediate alignment waypoint at (%.2f, %.2f, %.2f), ratio=%.2f",
                           intermediate_point.x(), intermediate_point.y(), intermediate_point.z(),
                           intermediate_waypoint_ratio_);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "Intermediate waypoint would be behind start position, skipping");
            }
        }

        // Add all original waypoints after intermediate point
        for (const auto& wp : adjusted_waypoints) {
            waypoints_with_intermediate.push_back(wp);
        }

        // Step 3: Prepare waypoints for B-spline generation
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : waypoints_with_intermediate) {
            all_points.push_back(wp);
        }

        log_manager_->infof("Global trajectory: start + %zu waypoints (including %s intermediate alignment point)",
                   waypoints_with_intermediate.size(),
                   waypoints_with_intermediate.size() > adjusted_waypoints.size() ? "1" : "0");

        // Step 4: Convert Vector3d to VectorXd for playground_bspline
        std::vector<Eigen::VectorXd> pts_vectorxd;
        if (!all_points.empty()) {
            // Start point: repeat 3 times
            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points.front().x(), all_points.front().y(), all_points.front().z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
            // Middle waypoints: add once for smooth trajectory
            for (size_t i = 1; i < all_points.size() - 1; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points[i].x(), all_points[i].y(), all_points[i].z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
            // End point: repeat 3 times
            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd pt_vectorxd(3);
                pt_vectorxd << all_points.back().x(), all_points.back().y(), all_points.back().z();
                pts_vectorxd.push_back(pt_vectorxd);
            }
        }

        // Safety check: B-spline requires at least 4 control points (order=3)
        if (pts_vectorxd.size() < 4) {
            RCLCPP_ERROR(node_->get_logger(),
                        "planGlobalTraj: Not enough points for B-spline (need >=4, got %zu). "
                        "This usually happens when A* returns too few waypoints.",
                        pts_vectorxd.size());
            return false;
        }

        // Step 5: Generate B-spline trajectory using playground_bspline
        std::vector<Eigen::VectorXd> b_pts = playground_bspline(pts_vectorxd);

        log_manager_->infof("Generated %zu B-spline points", b_pts.size());

        // Step 6: Convert back to Vector3d for trajectory generation
        std::vector<Eigen::Vector3d> sampled_points;
        for (const auto& b_pt : b_pts) {
            sampled_points.push_back(Eigen::Vector3d(b_pt.x(), b_pt.y(), b_pt.z()));
        }

        // Step 7: Convert to MINCO trajectory
        poly_traj::MinJerkOpt globalMJO;

        // Create waypoint trajectory using the sampled points
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        // Use adjusted waypoints' last point (not original waypoints)
        tailState << adjusted_waypoints.back(), end_vel, end_acc;
        
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
        
        log_manager_->infof("Successfully generated global trajectory with B-spline -> MINCO conversion");
        log_manager_->infof("Final trajectory: %d segments, duration: %.3f, max_vel: %.3f",
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

void PathManager::setFormationInfo(int drone_id, const std::string& formation_type,
                                   const std::vector<Eigen::Vector3d>& formation_pattern) {
    current_drone_id_ = drone_id;
    current_formation_type_ = formation_type;
    current_formation_pattern_ = formation_pattern;

    log_manager_->infof("PathManager: Set formation info - drone_id=%d, type=%s, pattern_size=%zu",
                drone_id, formation_type.c_str(), formation_pattern.size());
}

bool PathManager::EmergencyStop(const Eigen::Vector3d& stop_pos) {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;

    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_.setLocalTraj(stopMJO.getTraj(), rclcpp::Clock(RCL_ROS_TIME).now().seconds(), traj_.local_traj.drone_id);

    RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                stop_pos.x(), stop_pos.y(), stop_pos.z());
    if (log_manager_) {
        log_manager_->warnf("EMERGENCY STOP executed at position (%.2f, %.2f, %.2f)",
                           stop_pos.x(), stop_pos.y(), stop_pos.z());
    }

    return true;
}

double PathManager::computePathCurvature(const Eigen::Vector3d& p1,
                                        const Eigen::Vector3d& p2,
                                        const Eigen::Vector3d& p3) {
    // Compute curvature using three consecutive points
    // κ = 2 * area(triangle) / (|p1-p2| * |p2-p3| * |p3-p1|)

    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p2;

    double len1 = v1.norm();
    double len2 = v2.norm();

    if (len1 < 1e-6 || len2 < 1e-6) {
        return 0.0;  // Straight line or degenerate case
    }

    // Cross product gives twice the area of triangle
    Eigen::Vector3d cross = v1.cross(v2);
    double area = cross.norm() / 2.0;

    double len3 = (p3 - p1).norm();

    if (len3 < 1e-6) {
        return 0.0;
    }

    // Curvature (only magnitude, sign determined separately)
    double curvature = 2.0 * area / (len1 * len2 * len3);

    return curvature;
}

Eigen::Vector3d PathManager::computeLateralOffset(const Eigen::Vector3d& prev_point,
                                                  const Eigen::Vector3d& curr_point,
                                                  const Eigen::Vector3d& next_point,
                                                  double offset_distance) {
    // Compute the direction vectors
    Eigen::Vector3d v1 = curr_point - prev_point;
    Eigen::Vector3d v2 = next_point - curr_point;

    if (v1.norm() < 1e-6 || v2.norm() < 1e-6) {
        return curr_point;  // No offset for degenerate case
    }

    // Normalize
    v1.normalize();
    v2.normalize();

    // Compute the average tangent direction
    Eigen::Vector3d tangent = (v1 + v2).normalized();

    // Compute the normal vector (perpendicular to tangent in XY plane)
    // For 2D path in XY plane, normal is simply rotating tangent by 90 degrees
    Eigen::Vector3d normal(-tangent.y(), tangent.x(), 0.0);
    normal.normalize();

    // Apply offset: positive offset_distance means move to the right (outer line for CCW turn)
    Eigen::Vector3d offset_point = curr_point + normal * offset_distance;

    return offset_point;
}

std::vector<Eigen::Vector3d> PathManager::adjustWaypointsForFormation(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& start_pos) {

    // If no formation info set, return original waypoints
    if (current_drone_id_ < 0 || current_formation_pattern_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No formation info set, using original waypoints");
        return waypoints;
    }

    // Check if this is a line formation
    bool is_line_formation = (current_formation_type_.find("line") != std::string::npos);

    if (is_line_formation) {
        RCLCPP_INFO(node_->get_logger(),
                   "Line formation detected: using simple offset without outer/inner line");
        return adjustWaypointsForLineFormation(waypoints, start_pos);
    } else {
        RCLCPP_INFO(node_->get_logger(),
                   "Non-line formation (%s): using outer/inner line calculation",
                   current_formation_type_.c_str());
        return adjustWaypointsWithCurvature(waypoints, start_pos);
    }
}

std::vector<Eigen::Vector3d> PathManager::adjustWaypointsForLineFormation(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& start_pos) {

    // For line formations, waypoints already have offsets applied by FSM
    // Line formation is designed to be parallel to travel direction
    // No additional adjustment needed - just return original waypoints

    Eigen::Vector3d my_formation_offset = current_formation_pattern_[current_drone_id_];

    RCLCPP_INFO(node_->get_logger(),
                "Line formation: drone %d, formation offset (%.3f, %.3f, %.3f) - using original waypoints",
                current_drone_id_,
                my_formation_offset.x(), my_formation_offset.y(), my_formation_offset.z());

    RCLCPP_INFO(node_->get_logger(),
                "Line formation: returning %zu original waypoints without modification",
                waypoints.size());

    // Simply return the original waypoints
    return waypoints;
}

std::vector<Eigen::Vector3d> PathManager::adjustWaypointsWithCurvature(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& start_pos) {

    // For non-line formations: apply outer/inner line calculation with curvature

    Eigen::Vector3d my_formation_offset = current_formation_pattern_[current_drone_id_];

    // Use Y component as lateral offset (perpendicular to path)
    double lateral_offset = my_formation_offset.y();

    RCLCPP_INFO(node_->get_logger(),
                "Adjusting waypoints for drone %d with lateral offset %.3fm (formation Y: %.3f)",
                current_drone_id_, lateral_offset, my_formation_offset.y());

    std::vector<Eigen::Vector3d> adjusted_waypoints;
    adjusted_waypoints.reserve(waypoints.size());

    // Add start position
    std::vector<Eigen::Vector3d> all_points;
    all_points.push_back(start_pos);
    all_points.insert(all_points.end(), waypoints.begin(), waypoints.end());

    // For each waypoint, apply lateral offset based on path curvature
    for (size_t i = 0; i < all_points.size(); ++i) {
        Eigen::Vector3d adjusted_point;

        if (i == 0) {
            // First point: use next point for direction
            if (all_points.size() > 1) {
                Eigen::Vector3d dir = (all_points[1] - all_points[0]).normalized();
                Eigen::Vector3d normal(-dir.y(), dir.x(), 0.0);
                adjusted_point = all_points[0] + normal * lateral_offset;
            } else {
                adjusted_point = all_points[0];
            }
        } else if (i == all_points.size() - 1) {
            // Last point: use previous point for direction
            Eigen::Vector3d dir = (all_points[i] - all_points[i-1]).normalized();
            Eigen::Vector3d normal(-dir.y(), dir.x(), 0.0);
            adjusted_point = all_points[i] + normal * lateral_offset;
        } else {
            // Middle points: compute curvature and adjust offset
            double curvature = computePathCurvature(all_points[i-1], all_points[i], all_points[i+1]);

            // For curved sections, adjust the offset distance
            // Outer line (positive lateral_offset on CCW turn) needs larger radius
            // Inner line (negative lateral_offset on CCW turn) needs smaller radius
            double curvature_threshold = 0.01;  // Threshold to detect significant curves

            if (std::abs(curvature) > curvature_threshold) {
                // In curved section: apply additional offset based on curvature
                // This creates the outer/inner line effect
                double curvature_factor = 1.0 + curvature * 10.0;  // Scale factor
                adjusted_point = computeLateralOffset(
                    all_points[i-1], all_points[i], all_points[i+1],
                    lateral_offset * curvature_factor);

                RCLCPP_DEBUG(node_->get_logger(),
                            "Waypoint %zu: curvature=%.4f, factor=%.3f",
                            i, curvature, curvature_factor);
            } else {
                // Straight section: simple lateral offset
                adjusted_point = computeLateralOffset(
                    all_points[i-1], all_points[i], all_points[i+1],
                    lateral_offset);
            }
        }

        // Skip start position, only add waypoints
        if (i > 0) {
            adjusted_waypoints.push_back(adjusted_point);
        }
    }

    RCLCPP_INFO(node_->get_logger(),
                "Adjusted %zu waypoints with outer/inner line calculation",
                adjusted_waypoints.size());

    return adjusted_waypoints;
}

} // namespace path_manager
