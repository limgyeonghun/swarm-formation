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
        node_->declare_parameter("manager/sfc_progress", 7.0);
        node_->declare_parameter("manager/sfc_range", 3.0);
        node_->declare_parameter("manager/z_min", 0.0);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/sfc_progress", sfc_progress_);
        node_->get_parameter("manager/sfc_range", sfc_range_);
        node_->get_parameter("manager/z_min", z_min_);

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
        sfc_corridor_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/drone_" + std::to_string(drone_id) + "/sfc_corridor", 10);
        shortest_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_" + std::to_string(drone_id) + "/shortest_path", 10);
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
        log_manager_->infof("Planning global trajectory using RRT* + SFC corridor with %zu waypoints", waypoints.size());

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
        // Use direct geometry collision check (no ESDF/GridMap dependency)
        ObstacleQueryAdapter map_adapter;
        map_adapter.obstacles = &obstacle_centers_;
        map_adapter.safety_margin = 0.3;

        // Compute map bounds from waypoints (no GridMap dependency)
        map_lower_bound_ = start_pos;
        map_upper_bound_ = start_pos;

        // Extend bounds to include all waypoints with margin
        const double bound_margin = 10.0;
        for (const auto& pt : all_points) {
            for (int d = 0; d < 3; ++d) {
                map_lower_bound_(d) = std::min(map_lower_bound_(d), pt(d) - bound_margin);
                map_upper_bound_(d) = std::max(map_upper_bound_(d), pt(d) + bound_margin);
            }
        }

        // Enforce ground limit
        map_lower_bound_.z() = std::max(map_lower_bound_.z(), z_min_);

        log_manager_->infof("Map bounds: lower=(%.2f,%.2f,%.2f), upper=(%.2f,%.2f,%.2f)",
            map_lower_bound_.x(), map_lower_bound_.y(), map_lower_bound_.z(),
            map_upper_bound_.x(), map_upper_bound_.y(), map_upper_bound_.z());

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
            double cost = sfc_gen::planPath<ObstacleQueryAdapter>(
                all_points[seg], all_points[seg + 1],
                map_lower_bound_, map_upper_bound_,
                &map_adapter, 2.0, seg_path);

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
                full_route.push_back(seg_path[i]);
            }
        }

        log_manager_->infof("RRT* route: %zu waypoints", full_route.size());

        // Publish simple path for visualization
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        path_msg.header.frame_id = "world";
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

        // === STEP 3: Generate SFC corridor around RRT* path ===
        // Generate obstacle surface points from obstacle geometry (no GridMap/ESDF dependency)
        obstacle_points_.clear();
        const double point_spacing = 0.2;  // Surface point spacing
        const double z_range = 2.0;        // Z range for 2.5D obstacles
        const double z_step = 0.5;

        for (const auto &obs : obstacle_centers_) {
            if (obs.shape == ObstacleShape::CIRCLE) {
                double radius = (obs.param1 > 0) ? obs.param1 : 0.5;
                double circumference = 2.0 * M_PI * radius;
                int num_angular = std::max(12, (int)(circumference / point_spacing));

                for (int ai = 0; ai < num_angular; ++ai) {
                    double angle = 2.0 * M_PI * ai / num_angular;
                    // Surface points at radius
                    for (double z = obs.center.z() - z_range; z <= obs.center.z() + z_range; z += z_step) {
                        obstacle_points_.push_back(Eigen::Vector3d(
                            obs.center.x() + radius * cos(angle),
                            obs.center.y() + radius * sin(angle),
                            z));
                    }
                    // Interior points for solid obstacle representation
                    for (double r = point_spacing; r < radius; r += point_spacing) {
                        obstacle_points_.push_back(Eigen::Vector3d(
                            obs.center.x() + r * cos(angle),
                            obs.center.y() + r * sin(angle),
                            obs.center.z()));
                    }
                }
                // Center point
                obstacle_points_.push_back(obs.center);

            } else if (obs.shape == ObstacleShape::RECTANGLE) {
                double half_w = obs.param1 / 2.0;
                double half_h = obs.param2 / 2.0;

                // Edge points
                for (double z = obs.center.z() - z_range; z <= obs.center.z() + z_range; z += z_step) {
                    // Top and bottom edges
                    for (double dx = -half_w; dx <= half_w; dx += point_spacing) {
                        obstacle_points_.push_back(Eigen::Vector3d(obs.center.x() + dx, obs.center.y() - half_h, z));
                        obstacle_points_.push_back(Eigen::Vector3d(obs.center.x() + dx, obs.center.y() + half_h, z));
                    }
                    // Left and right edges
                    for (double dy = -half_h; dy <= half_h; dy += point_spacing) {
                        obstacle_points_.push_back(Eigen::Vector3d(obs.center.x() - half_w, obs.center.y() + dy, z));
                        obstacle_points_.push_back(Eigen::Vector3d(obs.center.x() + half_w, obs.center.y() + dy, z));
                    }
                }
                // Interior fill at center z
                for (double dx = -half_w; dx <= half_w; dx += point_spacing) {
                    for (double dy = -half_h; dy <= half_h; dy += point_spacing) {
                        obstacle_points_.push_back(Eigen::Vector3d(
                            obs.center.x() + dx, obs.center.y() + dy, obs.center.z()));
                    }
                }
            }
        }

        log_manager_->infof("Collected %zu obstacle points for SFC generation", obstacle_points_.size());

        // Generate SFC corridors using FIRI
        double sfc_progress = sfc_progress_;
        double sfc_range = sfc_range_;

        if (obstacle_points_.empty()) {
            // No obstacles: create a single large corridor (bounding box)
            // Add dummy far-away points so FIRI has something to work with
            obstacle_points_.push_back(map_lower_bound_ - Eigen::Vector3d(1, 1, 1));
            obstacle_points_.push_back(map_upper_bound_ + Eigen::Vector3d(1, 1, 1));
        }

        global_hpolys_.clear();
        sfc_gen::convexCover(full_route, obstacle_points_,
                             map_lower_bound_, map_upper_bound_,
                             sfc_progress, sfc_range, global_hpolys_);
        sfc_gen::shortCut(global_hpolys_);

        log_manager_->infof("Generated %zu SFC corridor polytopes", global_hpolys_.size());

        if (global_hpolys_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "SFC corridor generation failed!");
            return false;
        }

        // === STEP 4: GCOPTER-style initial trajectory through corridor ===
        // 4a. Normalize H-polytope normals (GCOPTER requirement)
        PolyhedraH normHpolys = global_hpolys_;
        for (size_t i = 0; i < normHpolys.size(); i++)
        {
            const Eigen::ArrayXd norms =
                normHpolys[i].leftCols<3>().rowwise().norm();
            normHpolys[i].array().colwise() /= norms;
        }

        // 4b. Convert H-polytopes to V-polytopes and compute corridor overlaps
        PolyhedraV vPolytopes;
        if (!processCorridor(normHpolys, vPolytopes))
        {
            RCLCPP_ERROR(node_->get_logger(), "processCorridor failed! Using fallback linear path.");
            // Fallback: simple 2-piece trajectory
            poly_traj::MinJerkOpt globalMJO;
            Eigen::Matrix<double, 3, 3> headState, tailState;
            headState << start_pos, start_vel, start_acc;
            tailState << waypoints.back(), end_vel, end_acc;
            int piece_num = 2;
            Eigen::MatrixXd innerPts(3, 1);
            innerPts.col(0) = (start_pos + waypoints.back()) * 0.5;
            globalMJO.reset(headState, tailState, piece_num);
            Eigen::VectorXd time_vec(piece_num);
            double dist = (waypoints.back() - start_pos).norm();
            time_vec.setConstant(std::max(0.1, dist / 2.0 / max_vel_));
            globalMJO.generate(innerPts, time_vec);
            auto time_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            traj_.setGlobalTraj(globalMJO.getTraj(), time_now);
            traj_.setLocalTraj(globalMJO.getTraj(), time_now, traj_.local_traj.drone_id);
            simple_path_ = full_route;
            return true;
        }

        // 4c. Find shortest path through corridor overlaps (L-BFGS optimized)
        const double smoothEps = 0.01;
        Eigen::Matrix3Xd shortPath;
        getShortestPath(start_pos, waypoints.back(), vPolytopes, smoothEps, shortPath);

        log_manager_->infof("Shortest path through %d corridor overlaps computed", (int)(vPolytopes.size() / 2));

        // Publish SFC corridor and shortest path for visualization
        publishSFCCorridor(normHpolys);
        publishShortestPath(shortPath);

        // 4d. Determine piece count per polytope (GCOPTER-style: based on segment length)
        const int polyN = global_hpolys_.size();
        const double lengthPerPiece = 2.0;  // GCOPTER default: finer pieces for smoother trajectory
        const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
        Eigen::VectorXi pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
        pieceIdx.array() += 1;  // At least 1 piece per polytope
        int piece_num = pieceIdx.sum();

        log_manager_->infof("Piece allocation: %d total pieces across %d polytopes", piece_num, polyN);

        // 4e. Generate initial inner points and time allocation from shortest path
        const double allocSpeed = max_vel_ * 3.0;  // GCOPTER: 3x max_vel for initial allocation
        Eigen::Matrix3Xd innerPts;
        Eigen::VectorXd time_vec;
        setInitialFromPath(shortPath, allocSpeed, pieceIdx, innerPts, time_vec);

        // 4f. Build MINCO trajectory
        poly_traj::MinJerkOpt globalMJO;
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        tailState << waypoints.back(), end_vel, end_acc;

        globalMJO.reset(headState, tailState, piece_num);
        globalMJO.generate(innerPts, time_vec);

        auto time_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(globalMJO.getTraj(), time_now);

        // Store the route for local planning
        simple_path_ = full_route;

        log_manager_->infof("Global trajectory (GCOPTER-style): SFC(%zu) -> ShortestPath -> MINCO(%d pieces)",
                   global_hpolys_.size(), piece_num);
        log_manager_->infof("Initial trajectory: %d segments, duration: %.3f, max_vel: %.3f",
                   globalMJO.getTraj().getPieceNum(), globalMJO.getTraj().getTotalDuration(),
                   globalMJO.getTraj().getMaxVelRate());

        // === STEP 5: L-BFGS optimization within SFC corridor (single-shot) ===
        if (isOptimizerInitialized() && !global_hpolys_.empty())
        {
            // Pass SFC corridor to optimizer
            poly_traj_opt_->setSFCCorridor(global_hpolys_);

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
            }
            else
            {
                // Fallback: use initial MINCO trajectory as local trajectory
                log_manager_->warnf("L-BFGS optimization failed, using initial MINCO trajectory");
                double start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
                traj_.setLocalTraj(globalMJO.getTraj(), start_time, traj_.local_traj.drone_id);
            }
        }
        else
        {
            // No optimizer or no corridor: use initial MINCO trajectory directly
            log_manager_->warnf("No optimizer or SFC corridor, using initial MINCO trajectory");
            double start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            traj_.setLocalTraj(globalMJO.getTraj(), start_time, traj_.local_traj.drone_id);
        }

        log_manager_->infof("Final optimized trajectory set as local_traj (single-shot, no replan)");

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

bool PathManager::isMapReady(const Eigen::Vector3d& start_pos) const {
    // If SFC corridor is available, check if start position is within corridor
    if (!global_hpolys_.empty()) {
        return true;  // SFC corridor available, map is ready
    }

    // No SFC corridor yet — map is ready once obstacles are parsed
    return !obstacle_centers_.empty() || true;  // Always ready (obstacles are geometry-based)
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

// ============================================================
// GCOPTER-style shortest path through SFC corridor overlaps
// Ported from gcopter/gcopter.hpp (Zhepei Wang, MIT License)
// ============================================================

bool PathManager::processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs)
{
    const int sizeCorridor = hPs.size() - 1;
    vPs.clear();
    vPs.reserve(2 * sizeCorridor + 1);

    int nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    for (int i = 0; i < sizeCorridor; i++)
    {
        if (!geo_utils::enumerateVs(hPs[i], curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);

        curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
        curIH.topRows(hPs[i].rows()) = hPs[i];
        curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
        if (!geo_utils::enumerateVs(curIH, curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);
    }

    if (!geo_utils::enumerateVs(hPs.back(), curIV))
    {
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
}

double PathManager::costDistance(void *ptr,
                                 const Eigen::VectorXd &xi,
                                 Eigen::VectorXd &gradXi)
{
    void **dataPtrs = (void **)ptr;
    const double &dEps = *((const double *)(dataPtrs[0]));
    const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
    const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
    const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

    double cost = 0.0;
    const int overlaps = vPolys.size() / 2;

    Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
    Eigen::Vector3d a, b, d;
    Eigen::VectorXd r;
    double smoothedDistance;
    for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
    {
        a = i == 0 ? ini : b;
        if (i < overlaps)
        {
            k = vPolys[2 * i + 1].cols();
            Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
            r = q.normalized().head(k - 1);
            b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                vPolys[2 * i + 1].col(0);
        }
        else
        {
            b = fin;
        }

        d = b - a;
        smoothedDistance = sqrt(d.squaredNorm() + dEps);
        cost += smoothedDistance;

        if (i < overlaps)
        {
            gradP.col(i) += d / smoothedDistance;
        }
        if (i > 0)
        {
            gradP.col(i - 1) -= d / smoothedDistance;
        }
    }

    Eigen::VectorXd unitQ;
    double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
    {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
        sqrNormQ = q.squaredNorm();
        invNormQ = 1.0 / sqrt(sqrNormQ);
        unitQ = q * invNormQ;
        gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                             unitQ.head(k - 1).array() * 2.0;
        gradQ(k - 1) = 0.0;
        gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

        sqrNormViolation = sqrNormQ - 1.0;
        if (sqrNormViolation > 0.0)
        {
            c = sqrNormViolation * sqrNormViolation;
            dc = 3.0 * c;
            c *= sqrNormViolation;
            cost += c;
            gradQ += dc * 2.0 * q;
        }
    }

    return cost;
}

void PathManager::getShortestPath(const Eigen::Vector3d &ini,
                                   const Eigen::Vector3d &fin,
                                   const PolyhedraV &vPolys,
                                   const double &smoothD,
                                   Eigen::Matrix3Xd &path)
{
    const int overlaps = vPolys.size() / 2;
    Eigen::VectorXi vSizes(overlaps);
    for (int i = 0; i < overlaps; i++)
    {
        vSizes(i) = vPolys[2 * i + 1].cols();
    }
    Eigen::VectorXd xi(vSizes.sum());
    for (int i = 0, j = 0; i < overlaps; i++)
    {
        xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
        j += vSizes(i);
    }

    double minDistance;
    void *dataPtrs[4];
    dataPtrs[0] = (void *)(&smoothD);
    dataPtrs[1] = (void *)(&ini);
    dataPtrs[2] = (void *)(&fin);
    dataPtrs[3] = (void *)(&vPolys);
    lbfgs::lbfgs_parameter_t shortest_path_params;
    shortest_path_params.past = 3;
    shortest_path_params.delta = 1.0e-3;
    shortest_path_params.g_epsilon = 1.0e-5;

    lbfgs::lbfgs_optimize(xi,
                           minDistance,
                           &PathManager::costDistance,
                           nullptr,
                           nullptr,
                           dataPtrs,
                           shortest_path_params);

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = ini;
    path.rightCols<1>() = fin;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
    {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        r = q.normalized().head(k - 1);
        path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                           vPolys[2 * i + 1].col(0);
    }
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

void PathManager::publishSFCCorridor(const PolyhedraH &hPolys)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // First, delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->get_clock()->now();
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < hPolys.size(); i++)
    {
        // Convert H-polytope to V-polytope for visualization
        Eigen::Matrix3Xd vPoly;
        if (!geo_utils::enumerateVs(hPolys[i], vPoly))
            continue;

        // Create wireframe from vertices using LINE_LIST
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "sfc_corridor";
        marker.id = i + 1;  // +1 because id=0 is delete_marker
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;  // Line width

        // Color: semi-transparent, different hue per polytope
        float hue = (float)i / std::max((int)hPolys.size(), 1);
        marker.color.r = 0.2f + 0.8f * std::max(0.0f, std::min(1.0f, std::abs(hue * 6.0f - 3.0f) - 1.0f));
        marker.color.g = 0.2f + 0.8f * std::max(0.0f, std::min(1.0f, 2.0f - std::abs(hue * 6.0f - 2.0f)));
        marker.color.b = 0.2f + 0.8f * std::max(0.0f, std::min(1.0f, 2.0f - std::abs(hue * 6.0f - 4.0f)));
        marker.color.a = 0.4f;

        marker.pose.orientation.w = 1.0;

        // Connect all vertex pairs as edges (convex hull wireframe)
        int nv = vPoly.cols();
        for (int a = 0; a < nv; a++)
        {
            for (int b = a + 1; b < nv; b++)
            {
                // Only draw edges shorter than a threshold (skip long diagonals)
                double edge_len = (vPoly.col(a) - vPoly.col(b)).norm();
                if (edge_len > 50.0) continue;  // Skip very long edges

                geometry_msgs::msg::Point p1, p2;
                p1.x = vPoly(0, a); p1.y = vPoly(1, a); p1.z = vPoly(2, a);
                p2.x = vPoly(0, b); p2.y = vPoly(1, b); p2.z = vPoly(2, b);
                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        if (!marker.points.empty())
            marker_array.markers.push_back(marker);
    }

    sfc_corridor_pub_->publish(marker_array);
    log_manager_->infof("Published SFC corridor visualization (%zu polytopes)", hPolys.size());
}

void PathManager::publishShortestPath(const Eigen::Matrix3Xd &path)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "shortest_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;  // Line width

    // Bright cyan color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.pose.orientation.w = 1.0;

    for (int i = 0; i < path.cols(); i++)
    {
        geometry_msgs::msg::Point p;
        p.x = path(0, i);
        p.y = path(1, i);
        p.z = path(2, i);
        marker.points.push_back(p);
    }

    shortest_path_pub_->publish(marker);
    log_manager_->infof("Published shortest path visualization (%d points)", (int)path.cols());
}

} // namespace path_manager
