#include "path_manager/path_manager.h"
#include "path_manager/polynomial_traj.h"

namespace path_manager
{

    PathManager::PathManager(rclcpp::Node::SharedPtr node)
        : node_(node),
          max_vel_(-1.0),
          max_acc_(-1.0),
          is_optimizer_initialized_(false),
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
        node_->declare_parameter("manager/length_per_piece", 3.0);
        node_->declare_parameter("manager/threat_weight", 10.0);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/threat_weight", threat_weight_);

        // Optional precomputed terrain ESDF file.
        // save: after first successful buildSDFForBounds, dump to this path.
        // load: if set and file exists, skip voxelization and load directly.
        node_->declare_parameter("manager/save_terrain_esdf", std::string());
        node_->declare_parameter("manager/load_terrain_esdf", std::string());
        node_->get_parameter("manager/save_terrain_esdf", save_terrain_esdf_path_);
        node_->get_parameter("manager/load_terrain_esdf", load_terrain_esdf_path_);

        // Parse threat zones: [cx, cy, cz, detection_range, max_threat_level, ...]
        node_->declare_parameter("threat_zones", std::vector<double>{});
        std::vector<double> tz_params;
        node_->get_parameter("threat_zones", tz_params);
        log_manager_->infof("Threat zone params size: %zu", tz_params.size());
        if (tz_params.size() >= 5 && tz_params.size() % 5 == 0) {
            for (size_t ti = 0; ti < tz_params.size(); ti += 5) {
                ThreatZone tz;
                tz.center = Eigen::Vector3d(tz_params[ti], tz_params[ti+1], tz_params[ti+2]);
                tz.detection_range = tz_params[ti+3];
                tz.max_threat_level = tz_params[ti+4];
                threat_zones_.push_back(tz);
                log_manager_->infof("  ThreatZone #%zu: center=(%.1f,%.1f,%.1f) detect=%.1f threat=%.1f",
                    threat_zones_.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.detection_range, tz.max_threat_level);
            }
            log_manager_->infof("Loaded %zu threat zones (threat_weight=%.1f)", threat_zones_.size(), threat_weight_);
        } else if (tz_params.empty()) {
            log_manager_->infof("No threat zones configured");
        } else {
            log_manager_->warnf("Invalid threat_zones param size: %zu (must be multiple of 5)", tz_params.size());
        }

        node_->declare_parameter("obstacles", std::vector<double>{});
        std::vector<double> obstacle_params;
        node_->get_parameter("obstacles", obstacle_params);

        // Parse obstacles with flexible format:
        // Basic: [x, y, z] - uses default inflation
        // Circle: [x, y, z, 0, radius]
        // Rectangle: [x, y, z, 1, width, height]
        size_t i = 0;
        while (i < obstacle_params.size())
        {
            if (i + 2 >= obstacle_params.size()) break;

            Eigen::Vector3d center(obstacle_params[i], obstacle_params[i + 1], obstacle_params[i + 2]);

            if (i + 3 < obstacle_params.size())
            {
                int shape_type = static_cast<int>(obstacle_params[i + 3]);

                if (shape_type == 0 && i + 4 < obstacle_params.size())  // CIRCLE
                {
                    double radius = obstacle_params[i + 4];
                    obstacle_centers_.emplace_back(center, radius);
                    i += 5;
                }
                else if (shape_type == 1 && i + 5 < obstacle_params.size())  // RECTANGLE
                {
                    double width = obstacle_params[i + 4];
                    double height = obstacle_params[i + 5];
                    obstacle_centers_.emplace_back(center, width, height);
                    i += 6;
                }
                else
                {
                    obstacle_centers_.emplace_back(center);
                    i += 3;
                }
            }
            else
            {
                obstacle_centers_.emplace_back(center);
                i += 3;
            }
        }

        simple_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/drone_" + std::to_string(drone_id) + "/simple_path", 10);
        ctrl_points_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/drone_" + std::to_string(drone_id) + "/ctrl_points", 10);
        rrt_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_" + std::to_string(drone_id) + "/rrt_path", 10);
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

            poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();

            // Set LogManager for unified logging
            poly_traj_opt_->setLogManager(log_manager_);

            // Set parameters first to ensure node_ is initialized
            poly_traj_opt_->setParam(node_);
            poly_traj_opt_->setDroneId(traj_.local_traj.drone_id);

            // Wire SDF-based obstacle avoidance into the optimizer.
            poly_traj_opt_->setSDFManager(&sdf_manager_);
            poly_traj_opt_->setObstacleClearance(obstacle_clearance_);

            // Pass threat zones to optimizer for trajectory fine-tuning (2nd stage)
            if (!threat_zones_.empty()) {
                std::vector<ego_planner::ThreatZone> opt_zones;
                for (const auto &tz : threat_zones_) {
                    ego_planner::ThreatZone oz;
                    oz.center = tz.center;
                    oz.detection_range = tz.detection_range;
                    oz.max_threat_level = tz.max_threat_level;
                    opt_zones.push_back(oz);
                }
                poly_traj_opt_->setThreatZones(opt_zones);
                log_manager_->infof("Passed %zu threat zones to optimizer", opt_zones.size());
            }

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

    bool PathManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                                     const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
        log_manager_->infof("Planning global trajectory with %zu waypoints", waypoints.size());
        auto t_total_start = std::chrono::steady_clock::now();

        if (waypoints.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "planGlobalTraj: No waypoints provided!");
            return false;
        }

        // === STEP 1: Build waypoint sequence ===
        // Build segment list: start -> wp1 -> wp2 -> ... -> wpN
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : waypoints) {
            all_points.push_back(wp);
        }

        // === STEP 2: RRT* path planning through waypoints ===
        auto t_rrt_start = std::chrono::steady_clock::now();
        // SDF-based collision query. Threat zones stay separate.
        std::vector<path_planner::sdf::ThreatZoneLite> sdf_threat_zones;
        sdf_threat_zones.reserve(threat_zones_.size());
        for (const auto &tz : threat_zones_) {
            sdf_threat_zones.push_back({tz.center, tz.detection_range, tz.max_threat_level});
        }
        path_planner::sdf::SDFQueryAdapter map_adapter;
        map_adapter.sdf = &sdf_manager_;
        map_adapter.threat_zones = sdf_threat_zones.empty() ? nullptr : &sdf_threat_zones;
        map_adapter.safety_margin = obstacle_clearance_;
        map_adapter.threat_weight = threat_weight_;

        // Compute map bounds from waypoints
        map_lower_bound_ = start_pos;
        map_upper_bound_ = start_pos;

        // Extend bounds to include all waypoints with margin
        double bound_margin_xy = 10.0;

        // If threat zones exist, expand margin to allow routing around them
        for (const auto &tz : threat_zones_) {
            bound_margin_xy = std::max(bound_margin_xy, tz.detection_range + 5.0);
        }

        for (const auto& pt : all_points) {
            map_lower_bound_.x() = std::min(map_lower_bound_.x(), pt.x() - bound_margin_xy);
            map_lower_bound_.y() = std::min(map_lower_bound_.y(), pt.y() - bound_margin_xy);
            map_upper_bound_.x() = std::max(map_upper_bound_.x(), pt.x() + bound_margin_xy);
            map_upper_bound_.y() = std::max(map_upper_bound_.y(), pt.y() + bound_margin_xy);
        }

        // Also extend bounds to include threat zone coverage areas
        for (const auto &tz : threat_zones_) {
            double r = tz.detection_range + 5.0;
            map_lower_bound_.x() = std::min(map_lower_bound_.x(), tz.center.x() - r);
            map_lower_bound_.y() = std::min(map_lower_bound_.y(), tz.center.y() - r);
            map_upper_bound_.x() = std::max(map_upper_bound_.x(), tz.center.x() + r);
            map_upper_bound_.y() = std::max(map_upper_bound_.y(), tz.center.y() + r);
        }

        // Z bounds: sample terrain along route to find max elevation
        double max_terrain_z = 0.0;
        if (terrain_data_.valid) {
            for (size_t i = 0; i < all_points.size() - 1; ++i) {
                Eigen::Vector3d dir = all_points[i+1] - all_points[i];
                double dist = dir.head<2>().norm();
                int n_samples = std::max(2, (int)(dist / 2.0));
                for (int s = 0; s <= n_samples; ++s) {
                    double t = (double)s / n_samples;
                    Eigen::Vector3d p = all_points[i] + t * dir;
                    // Also sample laterally
                    for (double offset : {-bound_margin_xy, 0.0, bound_margin_xy}) {
                        float elev = terrain_data_.getElevation(p.x() + offset, p.y());
                        if (elev > -1e10) max_terrain_z = std::max(max_terrain_z, (double)elev);
                        elev = terrain_data_.getElevation(p.x(), p.y() + offset);
                        if (elev > -1e10) max_terrain_z = std::max(max_terrain_z, (double)elev);
                    }
                }
            }
        }
        // Upper bound: terrain peak + vertical margin for trajectory room.
        map_upper_bound_.z() = max_terrain_z + 20.0;

        log_manager_->infof("Map bounds: lower=(%.2f,%.2f,%.2f), upper=(%.2f,%.2f,%.2f)",
            map_lower_bound_.x(), map_lower_bound_.y(), map_lower_bound_.z(),
            map_upper_bound_.x(), map_upper_bound_.y(), map_upper_bound_.z());

        // If save or load is requested, use the full-terrain bbox so the
        // cached ESDF is reusable across missions.
        const bool use_cache =
            !load_terrain_esdf_path_.empty() || !save_terrain_esdf_path_.empty();
        Eigen::Vector3d sdf_lo = map_lower_bound_;
        Eigen::Vector3d sdf_hi = map_upper_bound_;
        if (use_cache) {
            Eigen::Vector3d tlo, thi;
            if (computeTerrainBBox(&tlo, &thi)) {
                sdf_lo = tlo;
                sdf_hi = thi;
                log_manager_->infof("Using full-terrain bbox for SDF: lo=(%.2f,%.2f,%.2f) hi=(%.2f,%.2f,%.2f)",
                    sdf_lo.x(), sdf_lo.y(), sdf_lo.z(),
                    sdf_hi.x(), sdf_hi.y(), sdf_hi.z());
            } else {
                log_manager_->warnf("Terrain bbox unavailable; falling back to mission bbox for SDF");
            }
        }

        // Try loading a precomputed ESDF on the first plan.
        if (!sdf_loaded_from_file_ && !load_terrain_esdf_path_.empty()) {
            if (sdf_manager_.loadFromFile(load_terrain_esdf_path_, sdf_lo, sdf_hi)) {
                sdf_loaded_from_file_ = true;
                log_manager_->infof("SDF loaded from %s (skipping voxelization)",
                                    load_terrain_esdf_path_.c_str());
            } else {
                log_manager_->warnf("SDF load failed from %s; falling back to build",
                                    load_terrain_esdf_path_.c_str());
            }
        }

        // Build SDF (fallback or no load).
        if (!sdf_loaded_from_file_) {
            if (!buildSDFForBounds(sdf_lo, sdf_hi)) {
                RCLCPP_ERROR(node_->get_logger(), "SDF build failed");
                return false;
            }
            // Persist on first successful build if requested.
            if (!save_terrain_esdf_path_.empty()) {
                if (sdf_manager_.saveToFile(save_terrain_esdf_path_)) {
                    log_manager_->infof("SDF saved to %s",
                                        save_terrain_esdf_path_.c_str());
                    sdf_loaded_from_file_ = true;  // skip rebuild next mission
                }
            }
        }


        // SDF sanity probe at start/goal.
        {
            float d_start = sdf_manager_.getDistance(start_pos);
            float d_goal  = sdf_manager_.getDistance(waypoints.back());
            log_manager_->infof("SDF probe: start=%.3f m, goal=%.3f m (margin=%.2f)",
                                d_start, d_goal, obstacle_clearance_);
        }

        // Debug: check obstacle query at known obstacle positions
        for (const auto &obs : obstacle_centers_) {
            int q = map_adapter.query(obs.center);
            log_manager_->infof("Obstacle at (%.2f,%.2f,%.2f): query=%d (shape=%d, param1=%.2f)",
                obs.center.x(), obs.center.y(), obs.center.z(), q,
                (int)obs.shape, obs.param1);
        }

        std::vector<Eigen::Vector3d> full_route;
        full_route.push_back(start_pos);

        for (size_t seg = 0; seg < all_points.size() - 1; ++seg)
        {
            std::vector<Eigen::Vector3d> seg_path;
            double rrt_timeout = threat_zones_.empty() ? 2.0 : 5.0;
            double cost = sfc_gen::planPath<path_planner::sdf::SDFQueryAdapter>(
                all_points[seg], all_points[seg + 1],
                map_lower_bound_, map_upper_bound_,
                &map_adapter, rrt_timeout, seg_path);

            log_manager_->infof("RRT* segment %zu: cost=%.3f, path_size=%zu",
                seg, cost, seg_path.size());

            if (std::isinf(cost) || seg_path.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "RRT* failed for segment %zu: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                    seg, all_points[seg].x(), all_points[seg].y(), all_points[seg].z(),
                    all_points[seg+1].x(), all_points[seg+1].y(), all_points[seg+1].z());
                return false;
            }

            // Append path (skip first point of subsequent segments to avoid duplicates)
            for (size_t i = (seg == 0 ? 0 : 1); i < seg_path.size(); ++i)
            {
                if (!full_route.empty() &&
                    (full_route.back() - seg_path[i]).norm() < 1e-3) {
                    continue;
                }
                full_route.push_back(seg_path[i]);
            }
        }

        auto t_rrt_end = std::chrono::steady_clock::now();
        log_manager_->infof("RRT* route: %zu waypoints (%.1f ms)",
            full_route.size(),
            std::chrono::duration<double, std::milli>(t_rrt_end - t_rrt_start).count());
        for (size_t ri = 0; ri < full_route.size(); ++ri) {
            const auto &p = full_route[ri];
            log_manager_->infof("  RRT*[%zu]: (%.2f, %.2f, %.2f)", ri, p.x(), p.y(), p.z());
        }

        // Publish simple path for visualization
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        path_msg.header.frame_id = "map";
        for (const auto &point : full_route) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        simple_path_pub_->publish(path_msg);

        // Publish RRT* path as LINE_STRIP + SPHERE_LIST for debugging (cyan)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            line.ns = "rrt_path_line";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.25;
            line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;
            line.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            rrt_path_pub_->publish(line);

            visualization_msgs::msg::Marker dots;
            dots.header = line.header;
            dots.ns = "rrt_path_dots";
            dots.id = 1;
            dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            dots.action = visualization_msgs::msg::Marker::ADD;
            dots.pose.orientation.w = 1.0;
            dots.scale.x = 0.6; dots.scale.y = 0.6; dots.scale.z = 0.6;
            dots.color.r = 0.0; dots.color.g = 0.8; dots.color.b = 1.0; dots.color.a = 1.0;
            dots.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                dots.points.push_back(pt);
            }
            rrt_path_pub_->publish(dots);
        }

        // === STEP 3: SDF-based path shortening (replaces SFC + shortest-path) ===
        auto t_shorten_start = std::chrono::steady_clock::now();
        const std::vector<path_planner::sdf::ThreatZoneLite>* threats_for_short =
            sdf_threat_zones.empty() ? nullptr : &sdf_threat_zones;
        std::vector<Eigen::Vector3d> clean_path =
            path_planner::sdf::shortenPath(sdf_manager_, full_route,
                                            obstacle_clearance_,
                                            threats_for_short,
                                            sdf_voxel_size_);
        auto t_shorten_end = std::chrono::steady_clock::now();
        log_manager_->infof("Path shortening: %zu -> %zu waypoints (%.1f ms)",
            full_route.size(), clean_path.size(),
            std::chrono::duration<double, std::milli>(t_shorten_end - t_shorten_start).count());
        if (clean_path.size() < 2) {
            RCLCPP_ERROR(node_->get_logger(), "clean_path too short");
            return false;
        }

        // === STEP 4: MINCO initial trajectory from clean_path ===
        auto t_minco_start = std::chrono::steady_clock::now();
        const int seg_count = static_cast<int>(clean_path.size()) - 1;
        Eigen::VectorXi pieceIdx(seg_count);
        for (int i = 0; i < seg_count; ++i) {
            double seg_len = (clean_path[i + 1] - clean_path[i]).norm();
            pieceIdx(i) = std::max(1, static_cast<int>(std::ceil(seg_len / length_per_piece_)));
        }
        int piece_num = pieceIdx.sum();

        Eigen::Matrix3Xd clean_mat(3, clean_path.size());
        for (size_t i = 0; i < clean_path.size(); ++i) clean_mat.col(i) = clean_path[i];

        const double allocSpeed = max_vel_ * 3.0;
        Eigen::Matrix3Xd innerPts;
        Eigen::VectorXd time_vec;
        setInitialFromPath(clean_mat, allocSpeed, pieceIdx, innerPts, time_vec);

        Eigen::Vector3d approach_dir;
        if (clean_path.size() >= 2) {
            approach_dir = (clean_path.back() - clean_path[clean_path.size() - 2]).normalized();
        } else {
            approach_dir = (waypoints.back() - start_pos).normalized();
        }
        Eigen::Vector3d traj_end_vel = approach_dir * max_vel_;
        Eigen::Vector3d traj_end_acc = Eigen::Vector3d::Zero();

        poly_traj::MinJerkOpt globalMJO;
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        tailState << waypoints.back(), traj_end_vel, traj_end_acc;

        globalMJO.reset(headState, tailState, piece_num);
        globalMJO.generate(innerPts, time_vec);

        auto time_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(globalMJO.getTraj(), time_now);
        simple_path_ = full_route;

        log_manager_->infof("MINCO: pieces=%d duration=%.3f max_vel=%.3f",
            globalMJO.getTraj().getPieceNum(),
            globalMJO.getTraj().getTotalDuration(),
            globalMJO.getTraj().getMaxVelRate());

        auto t_minco_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] MINCO trajectory generation: %.1f ms",
            std::chrono::duration<double, std::milli>(t_minco_end - t_minco_start).count());

        // STEP 5: L-BFGS optimization with SDF gradient penalty.
        const bool run_optimizer = true;
        auto t_opt_start = std::chrono::steady_clock::now();
        if (run_optimizer && isOptimizerInitialized())
        {
            // Set control points from initial trajectory
            poly_traj::Trajectory initTraj = globalMJO.getTraj();
            Eigen::MatrixXd cps = globalMJO.getInitConstrainPoints(poly_traj_opt_->get_cps_num_prePiece_());
            poly_traj_opt_->setControlPoints(cps);

            // Prepare optimization inputs
            int PN = initTraj.getPieceNum();
            Eigen::MatrixXd all_pos = initTraj.getPositions();
            Eigen::MatrixXd optInnerPts = all_pos.block(0, 1, 3, PN - 1);

            // Run L-BFGS optimization (single shot, no replan)
            Eigen::MatrixXd optimal_points;
            bool use_formation = true;
            bool opt_success = poly_traj_opt_->OptimizeTrajectory_lbfgs(
                headState, tailState, optInnerPts, initTraj.getDurations(),
                optimal_points, use_formation);

            if (opt_success)
            {
                // Set optimized trajectory as local trajectory
                poly_traj::Trajectory optTraj = poly_traj_opt_->getMinJerkOptPtr()->getTraj();
                double start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
                traj_.setLocalTraj(optTraj, start_time, traj_.local_traj.drone_id);

                log_manager_->infof("L-BFGS optimization SUCCESS: duration=%.3f, max_vel=%.3f",
                    optTraj.getTotalDuration(), optTraj.getMaxVelRate());

                // === Control points 시각화 ===
                {
                    visualization_msgs::msg::MarkerArray arr;
                    const auto stamp = node_->get_clock()->now();

                    visualization_msgs::msg::Marker del;
                    del.action = visualization_msgs::msg::Marker::DELETEALL;
                    del.header.frame_id = "map";
                    del.header.stamp = stamp;
                    arr.markers.push_back(del);

                    for (int ci = 0; ci < optimal_points.cols(); ++ci) {
                        visualization_msgs::msg::Marker s;
                        s.header.frame_id = "map";
                        s.header.stamp = stamp;
                        s.ns = "ctrl_points";
                        s.id = ci;
                        s.type = visualization_msgs::msg::Marker::SPHERE;
                        s.action = visualization_msgs::msg::Marker::ADD;
                        s.pose.position.x = optimal_points(0, ci);
                        s.pose.position.y = optimal_points(1, ci);
                        s.pose.position.z = optimal_points(2, ci);
                        s.pose.orientation.w = 1.0;
                        s.scale.x = 0.4; s.scale.y = 0.4; s.scale.z = 0.4;
                        s.color.r = 1.0f; s.color.g = 0.3f; s.color.b = 1.0f; s.color.a = 1.0f;
                        arr.markers.push_back(s);

                        visualization_msgs::msg::Marker t;
                        t.header.frame_id = "map";
                        t.header.stamp = stamp;
                        t.ns = "ctrl_points_label";
                        t.id = ci;
                        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                        t.action = visualization_msgs::msg::Marker::ADD;
                        t.pose.position.x = optimal_points(0, ci);
                        t.pose.position.y = optimal_points(1, ci);
                        t.pose.position.z = optimal_points(2, ci) + 0.6;
                        t.pose.orientation.w = 1.0;
                        t.scale.z = 0.5;
                        t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 1.0f;
                        t.text = "cp" + std::to_string(ci);
                        arr.markers.push_back(t);
                    }
                    ctrl_points_pub_->publish(arr);
                    log_manager_->infof("Published %d control points", (int)optimal_points.cols());
                    for (int ci = 0; ci < optimal_points.cols(); ++ci) {
                        log_manager_->infof("  CP[%d]: (%.2f, %.2f, %.2f)",
                            ci, optimal_points(0, ci), optimal_points(1, ci), optimal_points(2, ci));
                    }
                }

                // Terrain collision is now handled implicitly by the SDF
                // penalty in the optimizer (phase 5). No post-check needed.
            }
            else
            {
                // Fallback: use initial MINCO trajectory as local trajectory.
                log_manager_->warnf("L-BFGS optimization failed, using initial MINCO trajectory");
                double start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
                traj_.setLocalTraj(globalMJO.getTraj(), start_time, traj_.local_traj.drone_id);
            }
        }
        else
        {
            // No optimizer: use initial MINCO trajectory directly.
            log_manager_->warnf("No optimizer, using initial MINCO trajectory");
            double start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            traj_.setLocalTraj(globalMJO.getTraj(), start_time, traj_.local_traj.drone_id);
        }

        auto t_opt_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] L-BFGS optimization: %.1f ms",
            std::chrono::duration<double, std::milli>(t_opt_end - t_opt_start).count());

        auto t_total_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] === TOTAL planGlobalTraj: %.1f ms ===",
            std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count());

        return true;
    }

bool PathManager::checkCollision(int drone_id)
{
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = std::max(my_traj_start_time, other_traj_start_time);
    double t_end = std::min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
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

bool PathManager::isMapReady(const Eigen::Vector3d& /*start_pos*/) const {
    // Phase 4: obstacles are always loaded from yaml, so map is always ready.
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

void PathManager::setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr &msg) {
    if (!msg || msg->layers.empty()) {
        log_manager_->warnf("Received empty terrain GridMap");
        return;
    }

    // Find elevation layer
    int elev_idx = -1;
    for (size_t i = 0; i < msg->layers.size(); ++i) {
        if (msg->layers[i] == "elevation") {
            elev_idx = static_cast<int>(i);
            break;
        }
    }
    if (elev_idx < 0) {
        log_manager_->warnf("Elevation layer not found in terrain GridMap");
        return;
    }

    const auto& elev_data = msg->data[elev_idx];
    if (elev_data.layout.dim.size() < 2) {
        log_manager_->warnf("Invalid terrain GridMap data layout");
        return;
    }

    terrain_data_.cols = elev_data.layout.dim[0].size;
    terrain_data_.rows = elev_data.layout.dim[1].size;
    terrain_data_.resolution = msg->info.resolution;
    terrain_data_.length_x = msg->info.length_x;
    terrain_data_.length_y = msg->info.length_y;
    terrain_data_.origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
    terrain_data_.origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;
    terrain_data_.center_x = msg->info.pose.position.x;
    terrain_data_.center_y = msg->info.pose.position.y;
    terrain_data_.elevation = elev_data.data;
    terrain_data_.valid = true;

    log_manager_->infof("Terrain data loaded: %dx%d, resolution=%.3f, origin=(%.2f,%.2f), center=(%.2f,%.2f)",
        terrain_data_.cols, terrain_data_.rows, terrain_data_.resolution,
        terrain_data_.origin_x, terrain_data_.origin_y,
        terrain_data_.center_x, terrain_data_.center_y);
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

    // 3D: Use cross product with world up vector
    Eigen::Vector3d up(0.0, 0.0, 1.0);
    Eigen::Vector3d normal = tangent.cross(up);

    // Handle case where tangent is parallel to up vector
    if (normal.norm() < 1e-6) {
        // Use alternative perpendicular vector
        Eigen::Vector3d alt_up(0.0, 1.0, 0.0);
        normal = tangent.cross(alt_up);
    }
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

    // Check for NONE mode - skip all formation adjustments
    bool is_none_mode = (current_formation_type_ == "none" || current_formation_type_ == "NONE");
    if (is_none_mode) {
        RCLCPP_INFO(node_->get_logger(),
                   "NONE mode detected: returning original waypoints without any formation adjustments");
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
                // Use computeLateralOffset for proper 3D handling
                // Create a virtual previous point by extrapolating backwards
                Eigen::Vector3d dir = (all_points[1] - all_points[0]).normalized();
                Eigen::Vector3d virtual_prev = all_points[0] - dir;
                adjusted_point = computeLateralOffset(virtual_prev, all_points[0], all_points[1], lateral_offset);
            } else {
                adjusted_point = all_points[0];
            }
        } else if (i == all_points.size() - 1) {
            // Last point: use previous point for direction
            // Create a virtual next point by extrapolating forwards
            Eigen::Vector3d dir = (all_points[i] - all_points[i-1]).normalized();
            Eigen::Vector3d virtual_next = all_points[i] + dir;
            adjusted_point = computeLateralOffset(all_points[i-1], all_points[i], virtual_next, lateral_offset);
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

void PathManager::setInitialFromPath(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
{
    const int sizeM = intervalNs.size();
    const int sizeN = intervalNs.sum();
    innerPoints.resize(3, sizeN - 1);
    timeAlloc.resize(sizeN);

    Eigen::Vector3d a, b, c;
    for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
    {
        l = intervalNs(i);
        a = path.col(i);
        b = path.col(i + 1);
        c = (b - a) / l;
        timeAlloc.segment(j, l).setConstant(c.norm() / speed);
        j += l;
        for (int m = 0; m < l; m++)
        {
            if (i > 0 || m > 0)
            {
                innerPoints.col(k++) = a + c * m;
            }
        }
    }
}
// Voxelize terrain + geometry obstacles into an occupancy grid and build ESDF.
// Threat zones are NOT included: they are handled as soft cost elsewhere.
bool PathManager::buildSDFForBounds(const Eigen::Vector3d &lo,
                                    const Eigen::Vector3d &hi)
{
    const double res = sdf_voxel_size_;
    Eigen::Vector3d ext = hi - lo;
    if ((ext.array() <= 0.0).any()) {
        log_manager_->warnf("buildSDFForBounds: invalid bounds");
        return false;
    }
    int nx = std::max(8, (int)std::ceil(ext.x() / res));
    int ny = std::max(8, (int)std::ceil(ext.y() / res));
    int nz = std::max(8, (int)std::ceil(ext.z() / res));

    std::vector<uint8_t> occ((size_t)nx * ny * nz, 0);
    auto idx = [&](int xi, int yi, int zi) {
        return ((size_t)xi * ny + yi) * nz + zi;
    };

    // Terrain: voxels strictly below the surface are occupied.
    if (terrain_data_.valid) {
        for (int xi = 0; xi < nx; ++xi) {
            double wx = lo.x() + (xi + 0.5) * res;
            for (int yi = 0; yi < ny; ++yi) {
                double wy = lo.y() + (yi + 0.5) * res;
                float elev = terrain_data_.getElevation(wx, wy);
                if (elev <= -1e10) continue;
                int zi_max = std::min(nz, (int)std::ceil((elev - lo.z()) / res));
                for (int zi = 0; zi < zi_max; ++zi) {
                    occ[idx(xi, yi, zi)] = 1;
                }
            }
        }
    }

    // Geometry obstacles.
    for (const auto &obs : obstacle_centers_) {
        if (obs.shape == ObstacleShape::CIRCLE) {
            double r = (obs.param1 > 0) ? obs.param1 : 0.5;
            int xi_lo = std::max(0,   (int)std::floor((obs.center.x() - r - lo.x()) / res));
            int xi_hi = std::min(nx,  (int)std::ceil ((obs.center.x() + r - lo.x()) / res));
            int yi_lo = std::max(0,   (int)std::floor((obs.center.y() - r - lo.y()) / res));
            int yi_hi = std::min(ny,  (int)std::ceil ((obs.center.y() + r - lo.y()) / res));
            for (int xi = xi_lo; xi < xi_hi; ++xi) {
                double wx = lo.x() + (xi + 0.5) * res;
                for (int yi = yi_lo; yi < yi_hi; ++yi) {
                    double wy = lo.y() + (yi + 0.5) * res;
                    double dx = wx - obs.center.x();
                    double dy = wy - obs.center.y();
                    if (dx * dx + dy * dy > r * r) continue;
                    for (int zi = 0; zi < nz; ++zi) occ[idx(xi, yi, zi)] = 1;
                }
            }
        } else if (obs.shape == ObstacleShape::RECTANGLE) {
            double hw = obs.param1 * 0.5;
            double hh = obs.param2 * 0.5;
            int xi_lo = std::max(0,  (int)std::floor((obs.center.x() - hw - lo.x()) / res));
            int xi_hi = std::min(nx, (int)std::ceil ((obs.center.x() + hw - lo.x()) / res));
            int yi_lo = std::max(0,  (int)std::floor((obs.center.y() - hh - lo.y()) / res));
            int yi_hi = std::min(ny, (int)std::ceil ((obs.center.y() + hh - lo.y()) / res));
            for (int xi = xi_lo; xi < xi_hi; ++xi) {
                for (int yi = yi_lo; yi < yi_hi; ++yi) {
                    for (int zi = 0; zi < nz; ++zi) occ[idx(xi, yi, zi)] = 1;
                }
            }
        }
    }

    if (!sdf_manager_.isInitialized()) {
        sdf_manager_.initialize(res);
    }
    bool ok = sdf_manager_.buildFromVoxels(occ.data(), nx, ny, nz, lo);
    if (ok) {
        log_manager_->infof("SDF built: shape=(%d,%d,%d) voxel=%.2fm blocks=%zu",
            nx, ny, nz, res, sdf_manager_.numAllocatedBlocks());
    }
    return ok;
}

bool PathManager::computeTerrainBBox(Eigen::Vector3d* lo, Eigen::Vector3d* hi)
{
    if (!terrain_data_.valid) return false;
    if (terrain_bbox_computed_) {
        *lo = terrain_bbox_lo_;
        *hi = terrain_bbox_hi_;
        return true;
    }

    // XY: walk the 4 corners of the terrain grid through terrainToWorld to
    //     get the correct world-frame bounds after the X-mirror / rotation.
    const int cols = terrain_data_.cols;
    const int rows = terrain_data_.rows;
    if (cols <= 0 || rows <= 0) return false;

    std::vector<Eigen::Vector3d> corners = {
        terrain_data_.terrainToWorld(0,        0,        0.0f),
        terrain_data_.terrainToWorld(cols - 1, 0,        0.0f),
        terrain_data_.terrainToWorld(0,        rows - 1, 0.0f),
        terrain_data_.terrainToWorld(cols - 1, rows - 1, 0.0f),
    };
    double min_x = corners[0].x(), max_x = corners[0].x();
    double min_y = corners[0].y(), max_y = corners[0].y();
    for (const auto& c : corners) {
        min_x = std::min(min_x, c.x()); max_x = std::max(max_x, c.x());
        min_y = std::min(min_y, c.y()); max_y = std::max(max_y, c.y());
    }

    // Z: scan all elevation samples for the actual peak, add flight headroom.
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (float e : terrain_data_.elevation) {
        if (!std::isfinite(e)) continue;
        min_z = std::min(min_z, (double)e);
        max_z = std::max(max_z, (double)e);
    }
    if (!std::isfinite(min_z) || !std::isfinite(max_z)) {
        min_z = 0.0; max_z = 0.0;
    }
    const double z_headroom = 30.0;  // vertical margin above peak for traj room

    terrain_bbox_lo_ = Eigen::Vector3d(min_x, min_y, std::min(0.0, min_z));
    terrain_bbox_hi_ = Eigen::Vector3d(max_x, max_y, max_z + z_headroom);
    terrain_bbox_computed_ = true;
    *lo = terrain_bbox_lo_;
    *hi = terrain_bbox_hi_;
    return true;
}

} // namespace path_manager
