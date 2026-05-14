#include "path_manager/path_manager.h"
#include "path_manager/polynomial_traj.h"
#include <chrono>
#include <fstream>
#include <string>

namespace path_manager
{

// Read VmRSS from /proc/self/status, KB.
static long pmReadVmRssKB() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            long kb = -1;
            sscanf(line.c_str(), "VmRSS: %ld", &kb);
            return kb;
        }
    }
    return -1;
}

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
        node_->declare_parameter("manager/risk_weight", 10.0);
        node_->declare_parameter("manager/astar_step_size", 1.0);
        node_->declare_parameter("manager/sdf_voxel_size", 1.0);
        node_->declare_parameter("manager/ground_height", -0.1);
        node_->declare_parameter("manager/virtual_ceil_height", -0.1);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/risk_weight", risk_weight_);
        node_->get_parameter("manager/astar_step_size", astar_step_size_);
        node_->get_parameter("manager/sdf_voxel_size", sdf_voxel_size_);
        node_->get_parameter("manager/ground_height", ground_height_);
        node_->get_parameter("manager/virtual_ceil_height", virtual_ceil_height_);

        // Optional precomputed terrain ESDF file.
        // save: after first successful buildSDFForBounds, dump to this path.
        // load: if set and file exists, skip voxelization and load directly.
        node_->declare_parameter("manager/save_terrain_esdf", std::string());
        node_->declare_parameter("manager/load_terrain_esdf", std::string());
        node_->get_parameter("manager/save_terrain_esdf", save_terrain_esdf_path_);
        node_->get_parameter("manager/load_terrain_esdf", load_terrain_esdf_path_);

        // Parse risk zones: [cx, cy, cz, sensing_range, max_risk_level, ...]
        node_->declare_parameter("risk_zones", std::vector<double>{});
        std::vector<double> tz_params;
        node_->get_parameter("risk_zones", tz_params);
        log_manager_->infof("Risk zone params size: %zu", tz_params.size());
        if (tz_params.size() >= 5 && tz_params.size() % 5 == 0) {
            for (size_t ti = 0; ti < tz_params.size(); ti += 5) {
                RiskZone tz;
                tz.center = Eigen::Vector3d(tz_params[ti], tz_params[ti+1], tz_params[ti+2]);
                tz.sensing_range = tz_params[ti+3];
                tz.max_risk_level = tz_params[ti+4];
                risk_zones_.push_back(tz);
                log_manager_->infof("  RiskZone #%zu: center=(%.1f,%.1f,%.1f) range=%.1f risk=%.1f",
                    risk_zones_.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.sensing_range, tz.max_risk_level);
            }
            log_manager_->infof("Loaded %zu risk zones (risk_weight=%.1f)", risk_zones_.size(), risk_weight_);
        } else if (tz_params.empty()) {
            log_manager_->infof("No risk zones configured");
        } else {
            log_manager_->warnf("Invalid risk_zones param size: %zu (must be multiple of 5)", tz_params.size());
        }

        node_->declare_parameter("obstacles", std::vector<double>{});
        std::vector<double> obstacle_params;
        node_->get_parameter("obstacles", obstacle_params);

        // Parse obstacles with flexible format:
        // Basic: [x, y, z] - uses default inflation
        // Circle: [x, y, z, 0, radius]
        // Rectangle: [x, y, z, 1, width, height]
        // Primary obstacle format — 7 fixed fields per entry (ambiguity-free):
        //   Circle:    [cx, cy, cz, 0, radius, 0,      height]
        //   Rectangle: [cx, cy, cz, 1, width,  length, height]
        // `height == 0` means infinite column (legacy semantics).
        //
        // Legacy accepted:
        //   [x, y, z]  — plain point (also used as "no-obstacle" sentinel)
        //
        // Entries are dispatched by peeking at obstacle_params[i + 3]: a
        // recognized shape_type (0 or 1) starts a 7-field record; anything
        // else (including list end) falls back to the 3-field legacy form.
        constexpr size_t kObsFields = 7;
        size_t i = 0;
        while (i + 3 <= obstacle_params.size()) {
            Eigen::Vector3d center(obstacle_params[i + 0],
                                   obstacle_params[i + 1],
                                   obstacle_params[i + 2]);

            const bool have_shape_field = (i + 3 < obstacle_params.size());
            const int shape_type = have_shape_field
                ? static_cast<int>(obstacle_params[i + 3])
                : -1;
            const bool is_full_record =
                have_shape_field &&
                (shape_type == 0 || shape_type == 1) &&
                (i + kObsFields <= obstacle_params.size());

            if (is_full_record) {
                const double p1     = obstacle_params[i + 4];
                const double p2     = obstacle_params[i + 5];
                const double height = obstacle_params[i + 6];
                if (shape_type == 0) {  // CIRCLE: p1=radius, p2 unused
                    if (height > 0.0) {
                        obstacle_centers_.emplace_back(center, p1, height, true);
                    } else {
                        obstacle_centers_.emplace_back(center, p1);
                    }
                } else {  // RECTANGLE: p1=width, p2=length
                    if (height > 0.0) {
                        obstacle_centers_.emplace_back(center, p1, p2, height);
                    } else {
                        obstacle_centers_.emplace_back(center, p1, p2);
                    }
                }
                i += kObsFields;
            } else {
                // Legacy 3-field entry: plain point.
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
        shorten_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_" + std::to_string(drone_id) + "/shorten_path", 10);
        init_minco_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_" + std::to_string(drone_id) + "/init_minco_path", 10);
        esdf_occ_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_" + std::to_string(drone_id) + "/esdf_occupied", 1);
        inner_pts_init_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/drone_" + std::to_string(drone_id) + "/inner_pts_init", 10);
        inner_pts_opt_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/drone_" + std::to_string(drone_id) + "/inner_pts_opt", 10);
        // TRANSIENT_LOCAL so RViz, joining late, still gets the latest set.
        rclcpp::QoS dyn_qos(1);
        dyn_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        dyn_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        dyn_obstacle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/drone_" + std::to_string(drone_id) + "/dynamic_obstacles", dyn_qos);
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
            poly_traj_opt_->setGroundHeight(ground_height_);
            poly_traj_opt_->setVirtualCeilHeight(virtual_ceil_height_);

            // Pass risk zones to optimizer for trajectory fine-tuning (2nd stage)
            if (!risk_zones_.empty()) {
                std::vector<ego_planner::RiskZone> opt_zones;
                for (const auto &tz : risk_zones_) {
                    ego_planner::RiskZone oz;
                    oz.center = tz.center;
                    oz.sensing_range = tz.sensing_range;
                    oz.max_risk_level = tz.max_risk_level;
                    opt_zones.push_back(oz);
                }
                poly_traj_opt_->setRiskZones(opt_zones);
                log_manager_->infof("Passed %zu risk zones to optimizer", opt_zones.size());
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

        // === STEP 2 preamble: build SDF-based auxiliary query adapter. ===
        // Kept for logging/debug of obstacle_centers_; A* uses sdf_manager_
        // directly via astar_.setSDF().
        // SDF-based collision query. Risk zones stay separate.
        std::vector<path_planner::sdf::RiskZoneLite> sdf_risk_zones;
        sdf_risk_zones.reserve(risk_zones_.size());
        for (const auto &tz : risk_zones_) {
            sdf_risk_zones.push_back({tz.center, tz.sensing_range, tz.max_risk_level});
        }
        path_planner::sdf::SDFQueryAdapter map_adapter;
        map_adapter.sdf = &sdf_manager_;
        map_adapter.risk_zones = sdf_risk_zones.empty() ? nullptr : &sdf_risk_zones;
        map_adapter.safety_margin = obstacle_clearance_;
        map_adapter.risk_weight = risk_weight_;

        // Compute map bounds from waypoints
        map_lower_bound_ = start_pos;
        map_upper_bound_ = start_pos;

        // Extend bounds to include all waypoints with margin
        double bound_margin_xy = 10.0;
        // Z margin below the lowest waypoint. Without this the map bottom
        // sits exactly on the start altitude, meaning obstacles anchored at
        // that altitude touch the map floor — their SDF gradient looks
        // asymmetric (no voxels below, voxels above), which tricks L-BFGS
        // into attempting a vertical-only escape that smoothness then blocks.
        const double bound_margin_z_below = 5.0;

        // If risk zones exist, expand margin to allow routing around them
        for (const auto &tz : risk_zones_) {
            bound_margin_xy = std::max(bound_margin_xy, tz.sensing_range + 5.0);
        }

        for (const auto& pt : all_points) {
            map_lower_bound_.x() = std::min(map_lower_bound_.x(), pt.x() - bound_margin_xy);
            map_lower_bound_.y() = std::min(map_lower_bound_.y(), pt.y() - bound_margin_xy);
            map_lower_bound_.z() = std::min(map_lower_bound_.z(), pt.z() - bound_margin_z_below);
            map_upper_bound_.x() = std::max(map_upper_bound_.x(), pt.x() + bound_margin_xy);
            map_upper_bound_.y() = std::max(map_upper_bound_.y(), pt.y() + bound_margin_xy);
        }

        // Also extend bounds to include risk zone coverage areas
        for (const auto &tz : risk_zones_) {
            double r = tz.sensing_range + 5.0;
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
        // Load/save of a precomputed terrain ESDF only makes sense when a
        // terrain GridMap has actually been received. Without terrain data,
        // fall back to mission-bbox voxelization and skip file I/O entirely.
        const bool has_terrain = terrain_data_.valid;
        const bool want_cache =
            !load_terrain_esdf_path_.empty() || !save_terrain_esdf_path_.empty();
        const bool use_cache = want_cache && has_terrain;
        if (want_cache && !has_terrain) {
            log_manager_->warnf("save/load_terrain_esdf set but no terrain loaded; "
                                "skipping ESDF file I/O for this plan.");
        }

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

        // Try loading a precomputed ESDF on the first plan (terrain must exist).
        if (use_cache && !sdf_loaded_from_file_ && !load_terrain_esdf_path_.empty()) {
            if (sdf_manager_.loadFromFile(load_terrain_esdf_path_, sdf_lo, sdf_hi)) {
                sdf_loaded_from_file_ = true;
                log_manager_->infof("SDF loaded from %s (skipping voxelization)",
                                    load_terrain_esdf_path_.c_str());
            } else {
                log_manager_->warnf("SDF load failed from %s; falling back to build",
                                    load_terrain_esdf_path_.c_str());
            }
        }

        // Build SDF (fallback or no cache).
        if (!sdf_loaded_from_file_) {
            if (!buildSDFForBounds(sdf_lo, sdf_hi)) {
                RCLCPP_ERROR(node_->get_logger(), "SDF build failed");
                return false;
            }
            // Persist on first successful build if requested (terrain must exist).
            if (use_cache && !save_terrain_esdf_path_.empty()) {
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

        // === ESDF occupancy visualization ===
        // Sample the ESDF on a coarse grid and publish occupied voxels as a
        // CUBE_LIST so the user can overlay them on the terrain mesh in RViz
        // to confirm terrain → SDF mapping.
        if (esdf_occ_pub_) {
            const double step = 1.0;
            Eigen::Vector3d lo = map_lower_bound_;
            Eigen::Vector3d hi = map_upper_bound_;
            visualization_msgs::msg::Marker cubes;
            cubes.header.frame_id = "map";
            cubes.header.stamp = node_->get_clock()->now();
            cubes.ns = "esdf_occupied";
            cubes.id = 0;
            cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
            cubes.action = visualization_msgs::msg::Marker::ADD;
            cubes.pose.orientation.w = 1.0;
            cubes.scale.x = step; cubes.scale.y = step; cubes.scale.z = step;
            cubes.color.r = 1.0f; cubes.color.g = 0.1f; cubes.color.b = 0.1f; cubes.color.a = 0.4f;
            for (double x = lo.x(); x <= hi.x(); x += step) {
                for (double y = lo.y(); y <= hi.y(); y += step) {
                    for (double z = lo.z(); z <= hi.z(); z += step) {
                        Eigen::Vector3d p(x, y, z);
                        float d = sdf_manager_.getDistance(p);
                        if (std::isfinite(d) && d < 0.0f) {
                            geometry_msgs::msg::Point pt;
                            pt.x = x; pt.y = y; pt.z = z;
                            cubes.points.push_back(pt);
                        }
                    }
                }
            }
            log_manager_->infof("ESDF viz: %zu occupied cubes (step=%.1fm)",
                                cubes.points.size(), step);
            esdf_occ_pub_->publish(cubes);
        }

        // Debug: check obstacle query at known obstacle positions
        for (const auto &obs : obstacle_centers_) {
            int q = map_adapter.query(obs.center);
            log_manager_->infof("Obstacle at (%.2f,%.2f,%.2f): query=%d (shape=%d, param1=%.2f)",
                obs.center.x(), obs.center.y(), obs.center.z(), q,
                (int)obs.shape, obs.param1);
        }

        // === STEP 2: 3D A* search + visibility-thinning simple_path ===
        // Bind SDF + risk zones to the A* front-end. A* collision check uses
        // the ESDF (distance < obstacle_clearance_ == blocked), and risk
        // cost is added to the A* g-score per visited cell.
        std::vector<path_planner::astar::RiskZoneLite> astar_risks;
        astar_risks.reserve(risk_zones_.size());
        for (const auto &tz : risk_zones_) {
            astar_risks.push_back({tz.center, tz.sensing_range, tz.max_risk_level});
        }
        Eigen::Vector3d map_size = map_upper_bound_ - map_lower_bound_;
        astar_.setLogManager(log_manager_);
        astar_.setSDF(&sdf_manager_, map_lower_bound_, map_size, sdf_voxel_size_);
        const std::vector<path_planner::astar::RiskZoneLite> *astar_tz_ptr =
            astar_risks.empty() ? nullptr : &astar_risks;
        astar_.setRiskZones(astar_tz_ptr);
        log_manager_->infof("[PM DBG] setRiskZones: %zu zones (ptr=%p) weight=%.3f",
            astar_risks.size(), (const void*)astar_tz_ptr, risk_weight_);
        // A* must see obstacles so the simple_path it returns is already an
        // avoidance path. Feeding that into MINCO makes the initial inner
        // points sit OUTSIDE the obstacle, and L-BFGS only has to smooth the
        // detour — no saddle problem. If A* is blinded (search_ignores=true)
        // the optimiser gets a straight line through the obstacle centre and
        // the cylinder's rotational symmetry pins it at a zero-gradient
        // saddle, which matches what main-branch would also suffer under the
        // same debug configuration.
        astar_.setObstacleMargin(obstacle_clearance_);
        astar_.setSearchIgnoresObstacles(false);
        astar_.setGroundHeight(ground_height_);
        astar_.setVirtualCeilHeight(virtual_ceil_height_);
        astar_.setRiskWeight(risk_weight_);

        // Size the A* search pool to cover the entire SDF so any detour is
        // reachable regardless of the start/goal pair. A* centers the pool
        // on the midpoint of each query; as long as pool_size ≥ sdf shape,
        // the full map is inside the search region. Allocated once on the
        // first plan and reused for all later missions.
        {
            Eigen::Vector3i sdf_shape = sdf_manager_.shape();
            if (sdf_shape.minCoeff() <= 0) {
                log_manager_->errorf("SDF shape not available, cannot size A* pool");
                return false;
            }
            Eigen::Vector3i desired = sdf_shape;
            if (!astar_initialized_) {
                astar_pool_size_ = desired;
                astar_.initGridMap(astar_pool_size_);
                astar_initialized_ = true;
                log_manager_->infof("A* pool allocated: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            } else if (desired != astar_pool_size_) {
                astar_pool_size_ = desired;
                astar_.resizePool(astar_pool_size_);
                log_manager_->infof("A* pool resized: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            }
        }

        auto t_astar_start = std::chrono::steady_clock::now();
        std::vector<Eigen::Vector3d> full_route;
        full_route.push_back(start_pos);
        for (size_t seg = 0; seg < all_points.size() - 1; ++seg)
        {
            std::vector<Eigen::Vector3d> seg_path =
                astar_.astarSearchAndGetSimplePath(
                    astar_step_size_, all_points[seg], all_points[seg + 1],
                    traj_.local_traj.drone_id);

            log_manager_->infof("A* segment %zu: simple_path_size=%zu",
                seg, seg_path.size());

            if (seg_path.size() < 2)
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "A* failed for segment %zu: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                    seg, all_points[seg].x(), all_points[seg].y(), all_points[seg].z(),
                    all_points[seg+1].x(), all_points[seg+1].y(), all_points[seg+1].z());
                return false;
            }

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
        log_manager_->infof("A* route: %zu waypoints (%.1f ms)",
            full_route.size(),
            std::chrono::duration<double, std::milli>(t_rrt_end - t_astar_start).count());
        // DEBUG: annotate each A* waypoint with per-zone distance and the
        // Gaussian risk_cost that getRiskCost() would return there. If
        // a waypoint sits inside a zone but its cost is ~0 we know the
        // risk data passed to A* is wrong. If cost is huge but A* still
        // picked the waypoint we know the detour weight vs heuristic is off.
        for (size_t ri = 0; ri < full_route.size(); ++ri) {
            const auto &p = full_route[ri];
            double total = 0.0;
            std::string per_zone;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                const auto &tz = risk_zones_[zi];
                double dist = (p - tz.center).norm();
                double zone_cost = 0.0;
                if (dist < tz.sensing_range) {
                    double sigma = tz.sensing_range / 3.0;
                    double g = std::exp(-(dist * dist) / (2.0 * sigma * sigma));
                    zone_cost = tz.max_risk_level * g * risk_weight_;
                }
                total += zone_cost;
                char buf[64];
                std::snprintf(buf, sizeof(buf), " tz%zu(d=%.2f,c=%.2f)",
                              zi, dist, zone_cost);
                per_zone += buf;
            }
            log_manager_->infof("  A*[%zu]: (%.2f, %.2f, %.2f) total_Risk=%.3f%s",
                ri, p.x(), p.y(), p.z(), total, per_zone.c_str());
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

        // === STEP 3: Corner-adaptive densification of the A* shortcut. ===
        // A single uniform spacing cannot satisfy both requirements at
        // once: small spacing keeps the trajectory glued to the A* polyline
        // (stiff), large spacing lets MINCO's 5th-order polynomials
        // overshoot at direction changes (loops).  We split the spacing
        // into two regimes based on how sharp each shortcut vertex is:
        //
        //   * near a sharp corner → dense (length_per_piece_) so a bunch
        //     of small piece-boundary kinks absorb the direction change
        //     without letting the polynomial curl back on itself;
        //   * in long straights    → coarse (length_per_piece_ × k) so
        //     L-BFGS has big, loosely-linked pieces to reshape freely
        //     around terrain / risks.
        //
        // Concretely, each shortcut segment is split by linear
        // interpolation, with the step chosen per position:
        //   step(α) = dense   if α is within `corner_band` of either end
        //                       AND an actual corner is there,
        //           = coarse  otherwise.
        const double dense_step  = std::max(0.5, length_per_piece_);
        const double coarse_step = std::max(dense_step, length_per_piece_ * 4.0);
        const double corner_band = 6.0 * dense_step;   // m around each corner
        const double corner_angle_thresh_deg = 20.0;   // "sharp" if ≥ this

        auto is_sharp_corner = [&](size_t i) -> bool {
            if (i == 0 || i + 1 >= full_route.size()) return false;
            Eigen::Vector3d v_in  = (full_route[i]     - full_route[i - 1]).normalized();
            Eigen::Vector3d v_out = (full_route[i + 1] - full_route[i]    ).normalized();
            double c = std::clamp(v_in.dot(v_out), -1.0, 1.0);
            double ang_deg = std::acos(c) * 180.0 / M_PI;
            return ang_deg >= corner_angle_thresh_deg;
        };

        std::vector<Eigen::Vector3d> clean_path;
        clean_path.reserve(full_route.size() * 8);
        clean_path.push_back(full_route.front());

        for (size_t i = 0; i + 1 < full_route.size(); ++i) {
            const Eigen::Vector3d &a = full_route[i];
            const Eigen::Vector3d &b = full_route[i + 1];
            const double seg_len = (b - a).norm();
            if (seg_len < 1e-6) continue;

            const bool corner_start = is_sharp_corner(i);
            const bool corner_end   = is_sharp_corner(i + 1);

            // Walk from a to b in variable-size steps.  We pick the step
            // length at the current distance-along-segment so corner bands
            // shrink it on both ends.
            double t = 0.0;
            while (t < seg_len - 1e-6) {
                double d_to_start = t;
                double d_to_end   = seg_len - t;
                bool near_start = corner_start && d_to_start < corner_band;
                bool near_end   = corner_end   && d_to_end   < corner_band;
                double step = (near_start || near_end) ? dense_step : coarse_step;
                double t_next = std::min(seg_len, t + step);
                double alpha  = t_next / seg_len;
                clean_path.push_back(a + alpha * (b - a));
                t = t_next;
            }
        }
        log_manager_->infof(
            "A* shortcut %zu pts → corner-adaptive %zu pts "
            "(dense %.2f m @ corners, coarse %.2f m on straights)",
            full_route.size(), clean_path.size(), dense_step, coarse_step);

        // Publish initial path for RViz (orange).
        if (shorten_path_pub_) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = node_->get_clock()->now();
            line.ns = "shorten_path";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.3;
            line.color.r = 1.0f; line.color.g = 0.5f; line.color.b = 0.0f; line.color.a = 1.0f;
            for (const auto &p : clean_path) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            shorten_path_pub_->publish(line);
        }

        if (clean_path.size() < 2) {
            log_manager_->errorf("clean_path too short");
            return false;
        }

        // === STEP 4: MINCO initial trajectory from clean_path ===
        // Swarm-Formation style: each shortcut vertex becomes one MINCO
        // piece boundary directly. clean_path was already densified so
        // every segment is ≤ length_per_piece_; there is no extra
        // per-segment splitting here. This keeps every piece roughly the
        // same length, which is what stops the 5th-order polynomial from
        // overshooting at direction changes (the loop/twist artefact).
        auto t_minco_start = std::chrono::steady_clock::now();

        // Degenerate single-segment path → insert a midpoint so MINCO
        // still has at least two pieces.
        if (static_cast<int>(clean_path.size()) < 3) {
            Eigen::Vector3d mid =
                0.5 * (clean_path.front() + clean_path.back());
            clean_path.insert(clean_path.begin() + 1, mid);
        }

        int piece_num = static_cast<int>(clean_path.size()) - 1;
        Eigen::MatrixXd innerPts(3, piece_num - 1);
        for (int i = 0; i < piece_num - 1; ++i) {
            innerPts.col(i) = clean_path[i + 1];
        }

        // Per-piece duration from segment length and max_vel (matches the
        // Swarm-Formation reference).
        const double des_vel = max_vel_;
        Eigen::VectorXd time_vec(piece_num);
        for (int i = 0; i < piece_num; ++i) {
            double seg_len = (clean_path[i + 1] - clean_path[i]).norm();
            time_vec(i) = std::max(0.05, seg_len / des_vel);
        }

        Eigen::Vector3d approach_dir =
            (clean_path.back() - clean_path[clean_path.size() - 2]).normalized();
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

        // Publish the initial (pre-L-BFGS) MINCO trajectory as a dense green
        // LINE_STRIP so the user can compare it to the optimized result.
        if (init_minco_pub_) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = node_->get_clock()->now();
            line.ns = "init_minco_path";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.3;
            line.color.r = 0.0f; line.color.g = 1.0f; line.color.b = 0.2f; line.color.a = 1.0f;
            const auto &initTraj = globalMJO.getTraj();
            const double dt = 0.1;
            const double T = initTraj.getTotalDuration();
            for (double t = 0.0; t < T; t += dt) {
                Eigen::Vector3d p = initTraj.getPos(t);
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            Eigen::Vector3d pe = initTraj.getPos(T);
            geometry_msgs::msg::Point pte;
            pte.x = pe.x(); pte.y = pe.y(); pte.z = pe.z();
            line.points.push_back(pte);
            init_minco_pub_->publish(line);
        }

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

            // Publish initial inner points (orange spheres) — MINCO piece
            // boundaries before L-BFGS starts.
            if (inner_pts_init_pub_ && optInnerPts.cols() > 0) {
                visualization_msgs::msg::MarkerArray arr;
                const auto stamp = node_->get_clock()->now();
                visualization_msgs::msg::Marker del;
                del.action = visualization_msgs::msg::Marker::DELETEALL;
                del.header.frame_id = "map";
                del.header.stamp = stamp;
                arr.markers.push_back(del);
                for (int i = 0; i < optInnerPts.cols(); ++i) {
                    visualization_msgs::msg::Marker s;
                    s.header.frame_id = "map";
                    s.header.stamp = stamp;
                    s.ns = "inner_pts_init";
                    s.id = i;
                    s.type = visualization_msgs::msg::Marker::SPHERE;
                    s.action = visualization_msgs::msg::Marker::ADD;
                    s.pose.position.x = optInnerPts(0, i);
                    s.pose.position.y = optInnerPts(1, i);
                    s.pose.position.z = optInnerPts(2, i);
                    s.pose.orientation.w = 1.0;
                    s.scale.x = 0.6; s.scale.y = 0.6; s.scale.z = 0.6;
                    s.color.r = 1.0f; s.color.g = 0.55f; s.color.b = 0.0f; s.color.a = 0.9f;
                    arr.markers.push_back(s);
                }
                inner_pts_init_pub_->publish(arr);
                log_manager_->infof("Published %d initial inner points (orange)",
                                    (int)optInnerPts.cols());
            }

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

                // Publish optimized inner points (yellow spheres with labels)
                // — same piece boundaries after L-BFGS has moved them.
                if (inner_pts_opt_pub_) {
                    int PNo = optTraj.getPieceNum();
                    Eigen::MatrixXd all_pos_opt = optTraj.getPositions();
                    if (PNo >= 2) {
                        Eigen::MatrixXd optInnerOpt = all_pos_opt.block(0, 1, 3, PNo - 1);
                        visualization_msgs::msg::MarkerArray arr;
                        const auto stamp = node_->get_clock()->now();
                        visualization_msgs::msg::Marker del;
                        del.action = visualization_msgs::msg::Marker::DELETEALL;
                        del.header.frame_id = "map";
                        del.header.stamp = stamp;
                        arr.markers.push_back(del);
                        for (int i = 0; i < optInnerOpt.cols(); ++i) {
                            visualization_msgs::msg::Marker s;
                            s.header.frame_id = "map";
                            s.header.stamp = stamp;
                            s.ns = "inner_pts_opt";
                            s.id = i;
                            s.type = visualization_msgs::msg::Marker::SPHERE;
                            s.action = visualization_msgs::msg::Marker::ADD;
                            s.pose.position.x = optInnerOpt(0, i);
                            s.pose.position.y = optInnerOpt(1, i);
                            s.pose.position.z = optInnerOpt(2, i);
                            s.pose.orientation.w = 1.0;
                            s.scale.x = 0.7; s.scale.y = 0.7; s.scale.z = 0.7;
                            s.color.r = 1.0f; s.color.g = 1.0f; s.color.b = 0.0f; s.color.a = 1.0f;
                            arr.markers.push_back(s);

                            visualization_msgs::msg::Marker lbl;
                            lbl.header.frame_id = "map";
                            lbl.header.stamp = stamp;
                            lbl.ns = "inner_pts_opt_label";
                            lbl.id = i;
                            lbl.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                            lbl.action = visualization_msgs::msg::Marker::ADD;
                            lbl.pose.position.x = optInnerOpt(0, i);
                            lbl.pose.position.y = optInnerOpt(1, i);
                            lbl.pose.position.z = optInnerOpt(2, i) + 0.9;
                            lbl.pose.orientation.w = 1.0;
                            lbl.scale.z = 0.5;
                            lbl.color.r = 1.0f; lbl.color.g = 1.0f; lbl.color.b = 1.0f; lbl.color.a = 1.0f;
                            lbl.text = "ip" + std::to_string(i);
                            arr.markers.push_back(lbl);
                        }
                        inner_pts_opt_pub_->publish(arr);
                        log_manager_->infof("Published %d optimized inner points (yellow)",
                                            (int)optInnerOpt.cols());
                        for (int i = 0; i < optInnerOpt.cols(); ++i) {
                            log_manager_->infof("  IP[%d]: (%.2f, %.2f, %.2f)",
                                i, optInnerOpt(0, i), optInnerOpt(1, i), optInnerOpt(2, i));
                        }
                    }
                }

                // Control-point visualisation removed. For km-scale missions
                // the CP count (≈ pieces × cps_num_prePiece) easily hits 10k,
                // which stalls RViz. Inner points (inner_pts_opt) carry the
                // same optimisation information and are orders of magnitude
                // fewer, so they cover the debugging need.

                // Terrain collision is now handled implicitly by the SDF
                // penalty in the optimizer (phase 5). No post-check needed.
            }
            else
            {
                log_manager_->errorf("L-BFGS optimization failed");
                return false;
            }
        }
        else
        {
            log_manager_->errorf("Optimizer not initialized");
            return false;
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

    // Eager-load the cached terrain ESDF as soon as terrain arrives, so the
    // dynamic-obstacle layer can accept clicks before any mission runs. Without
    // this, addDynamicSphere rejects with "SDF not built yet" until the first
    // planGlobalTraj() is invoked.
    if (!sdf_loaded_from_file_ && !load_terrain_esdf_path_.empty()) {
        Eigen::Vector3d lo, hi;
        if (computeTerrainBBox(&lo, &hi)) {
            const long rss_before = pmReadVmRssKB();
            auto t0 = std::chrono::steady_clock::now();
            if (!sdf_manager_.isInitialized()) sdf_manager_.initialize(sdf_voxel_size_);
            if (sdf_manager_.loadFromFile(load_terrain_esdf_path_, lo, hi)) {
                auto t1 = std::chrono::steady_clock::now();
                const long rss_after = pmReadVmRssKB();
                const double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                sdf_loaded_from_file_ = true;
                log_manager_->infof("[PM MEM] SDF eager-load: %.1f ms (RSS %ld -> %ld KB, delta=%ld KB = %.2f MB)",
                                    load_ms, rss_before, rss_after,
                                    rss_after - rss_before,
                                    (double)(rss_after - rss_before) / 1024.0);
                log_manager_->infof("SDF eagerly loaded from %s",
                                    load_terrain_esdf_path_.c_str());
            }
        }
    }
}

int PathManager::addDynamicSphere(const Eigen::Vector3d& center, double radius)
{
    if (!sdf_manager_.hasData()) {
        log_manager_->warnf("addDynamicSphere: SDF not built yet, ignoring "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -1;
    }
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kSphere;
    spec.center = center;
    const double d = 2.0 * radius;
    spec.size = Eigen::Vector3d(d, d, d);

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicSphere: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -1;
    }
    dyn_patch_ids_.push_back(id);
    dyn_patch_centers_.push_back(center);
    dyn_patch_radii_.push_back(radius);
    log_manager_->infof("Dynamic sphere added: id=%d center=(%.2f,%.2f,%.2f) r=%.2f, total=%zu",
                        id, center.x(), center.y(), center.z(), radius,
                        sdf_manager_.numActiveObstacles());
    publishDynamicObstacles();
    return id;
}

void PathManager::clearDynamicObstacles()
{
    sdf_manager_.clearObstacles();
    dyn_patch_ids_.clear();
    dyn_patch_centers_.clear();
    dyn_patch_radii_.clear();
    log_manager_->infof("Dynamic obstacles cleared");
    publishDynamicObstacles();
}

void PathManager::publishDynamicObstacles()
{
    if (!dyn_obstacle_pub_) return;
    visualization_msgs::msg::MarkerArray arr;

    // Single DELETEALL marker first so removed patches disappear in RViz.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = node_->now();
    clear_marker.ns = "dynamic_obstacles";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_marker);

    for (size_t i = 0; i < dyn_patch_centers_.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "dynamic_obstacles";
        m.id = dyn_patch_ids_[i];
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = dyn_patch_centers_[i].x();
        m.pose.position.y = dyn_patch_centers_[i].y();
        m.pose.position.z = dyn_patch_centers_[i].z();
        m.pose.orientation.w = 1.0;
        const double d = 2.0 * dyn_patch_radii_[i];
        m.scale.x = d; m.scale.y = d; m.scale.z = d;
        m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.6f;
        arr.markers.push_back(m);
    }
    dyn_obstacle_pub_->publish(arr);
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
                   "NONE mode found: returning original waypoints without any formation adjustments");
        return waypoints;
    }

    // Check if this is a line formation
    bool is_line_formation = (current_formation_type_.find("line") != std::string::npos);

    if (is_line_formation) {
        RCLCPP_INFO(node_->get_logger(),
                   "Line formation found: using simple offset without outer/inner line");
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
            double curvature_threshold = 0.01;  // Threshold to identify significant curves

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
// Risk zones are NOT included: they are handled as soft cost elsewhere.
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
    size_t terrain_occupied_voxels = 0;
    size_t terrain_valid_queries = 0;
    size_t terrain_invalid_queries = 0;
    float terrain_max_elev = -1e30f;
    float terrain_min_elev = 1e30f;
    if (terrain_data_.valid) {
        for (int xi = 0; xi < nx; ++xi) {
            double wx = lo.x() + (xi + 0.5) * res;
            for (int yi = 0; yi < ny; ++yi) {
                double wy = lo.y() + (yi + 0.5) * res;
                float elev = terrain_data_.getElevation(wx, wy);
                if (elev <= -1e10) { ++terrain_invalid_queries; continue; }
                ++terrain_valid_queries;
                terrain_max_elev = std::max(terrain_max_elev, elev);
                terrain_min_elev = std::min(terrain_min_elev, elev);
                int zi_max = std::min(nz, (int)std::ceil((elev - lo.z()) / res));
                for (int zi = 0; zi < zi_max; ++zi) {
                    occ[idx(xi, yi, zi)] = 1;
                    ++terrain_occupied_voxels;
                }
            }
        }
        log_manager_->infof(
            "Terrain → SDF: valid_xy=%zu, invalid_xy=%zu, occupied_voxels=%zu, "
            "elev_range=[%.2f, %.2f]",
            terrain_valid_queries, terrain_invalid_queries, terrain_occupied_voxels,
            terrain_min_elev, terrain_max_elev);

        // Probe a few world points on the A* straight-line path (y≈78.5).
        const std::array<std::pair<double,double>, 5> probes = {{
            {100.0, 78.5}, {120.0, 78.5}, {141.4, 78.5}, {160.0, 78.5}, {180.0, 78.5}
        }};
        for (auto [px, py] : probes) {
            float e = terrain_data_.getElevation(px, py);
            log_manager_->infof("  terrain probe (%.1f, %.1f) -> elev=%.3f", px, py, e);
        }
    } else {
        log_manager_->infof("Terrain → SDF: terrain_data_ INVALID (not applied)");
    }

    // Geometry obstacles.
    auto compute_z_range = [&](const Obstacle &obs, int &zi_lo, int &zi_hi) {
        // z_extent == 0 means "infinite column" for back-compat.
        if (obs.z_extent <= 0.0) {
            zi_lo = 0;
            zi_hi = nz;
        } else {
            zi_lo = std::max(0,  (int)std::floor((obs.center.z() - lo.z()) / res));
            zi_hi = std::min(nz, (int)std::ceil ((obs.center.z() + obs.z_extent - lo.z()) / res));
        }
    };

    for (const auto &obs : obstacle_centers_) {
        int zi_lo, zi_hi;
        compute_z_range(obs, zi_lo, zi_hi);
        if (zi_hi <= zi_lo) continue;

        if (obs.shape == ObstacleShape::CIRCLE) {
            // Cube (L∞) inflation matching main-branch behaviour: the "radius"
            // is interpreted as a half-side. A perfect analytic cylinder
            // (dx² + dy² ≤ r²) has full rotational symmetry and produces
            // exactly-zero horizontal SDF gradients on its axis, which pins
            // L-BFGS at a saddle point. The cube breaks that symmetry
            // (corners are farther than face midpoints) and gives the
            // optimiser a usable horizontal descent direction even when the
            // trajectory lies on the obstacle's centre axis.
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
                    if (std::max(std::abs(dx), std::abs(dy)) > r) continue;
                    for (int zi = zi_lo; zi < zi_hi; ++zi) occ[idx(xi, yi, zi)] = 1;
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
                    for (int zi = zi_lo; zi < zi_hi; ++zi) occ[idx(xi, yi, zi)] = 1;
                }
            }
        }
    }

    // NOTE: ground and virtual ceiling are NOT voxelised here. Folding them
    // into the SDF would drag the clearance band above/below the actual
    // plane, so the trajectory would be pushed off a `-0.1` floor by up to
    // `obstacle_clearance` metres. Instead they are enforced as hard
    // half-space constraints inside the optimizer (sdfGradCostP), which
    // applies a unit upward/downward gradient only when the query point
    // crosses the plane — no clearance band, no lateral contamination.

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
