#include "path_manager/path_manager.h"

namespace path_manager
{

    PathManager::PathManager(rclcpp::Node::SharedPtr node)
        : node_(node),
          max_vel_(-1.0),
          max_acc_(-1.0),
          is_optimizer_initialized_(false),
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
        node_->declare_parameter("manager/risk_weight", 1.0);
        node_->declare_parameter("manager/risk_barrier", 100.0);
        node_->declare_parameter("manager/risk_smha_w", 2.0);
        node_->declare_parameter("manager/front_end", std::string("fm2"));
        node_->declare_parameter("manager/fm2_coarse_k", 4);
        node_->declare_parameter("manager/fm2_star", true);
        node_->declare_parameter("manager/astar_bypass_shortcut", false);
        node_->declare_parameter("manager/astar_step_size", 1.0);
        node_->declare_parameter("manager/sdf_voxel_size", 1.0);
        node_->declare_parameter("manager/ground_height", -0.1);
        node_->declare_parameter("manager/virtual_ceil_height", -0.1);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/length_per_piece", length_per_piece_);
        node_->get_parameter("manager/risk_weight", risk_weight_);
        node_->get_parameter("manager/risk_barrier", risk_barrier_);
        node_->get_parameter("manager/risk_smha_w", risk_smha_w_);
        node_->get_parameter("manager/front_end", front_end_str_);
        node_->get_parameter("manager/fm2_coarse_k", fm2_coarse_k_);
        node_->get_parameter("manager/fm2_star", fm2_star_);
        node_->get_parameter("manager/astar_bypass_shortcut", astar_bypass_shortcut_);
        node_->get_parameter("manager/astar_step_size", astar_step_size_);
        node_->get_parameter("manager/sdf_voxel_size", sdf_voxel_size_);
        node_->get_parameter("manager/ground_height", ground_height_);
        node_->get_parameter("manager/virtual_ceil_height", virtual_ceil_height_);

        // Building-mesh rendering of dynamic obstacles (see publishDynamicObstacles).
        node_->declare_parameter("obstacle_mesh_resource",
                                 std::string("package://mmp_visualization/meshes/building.dae"));
        node_->declare_parameter("obstacle_mesh_height", 60.0);
        node_->get_parameter("obstacle_mesh_resource", obstacle_mesh_resource_);
        node_->get_parameter("obstacle_mesh_height", obstacle_mesh_height_);

        // Visual mesh catalog: model name -> mesh resource + rendered native size [m]
        // (mesh base at z=0, XY centered). Add a model = drop a .dae in
        // mmp_visualization/meshes/ + one line here (measure size with trimesh).
        mesh_catalog_["building"] = { obstacle_mesh_resource_,
                                      Eigen::Vector3d(16.374, 13.358, 17.345) };
        mesh_catalog_["car"]      = { "package://mmp_visualization/meshes/car.dae",
                                      Eigen::Vector3d(17.679, 10.093, 4.620) };
        mesh_catalog_["ship"]     = { "package://mmp_visualization/meshes/simple_ship.dae",
                                      Eigen::Vector3d(9.972, 42.275, 10.234) };

        // ESDF cache resolution:
        //   manager/world  : map name (default "dokdo"); derives the .esdf path.
        //   manager/esdf_dir : .esdf read/write dir (relative to process CWD).
        //   manager/{save,load}_terrain_esdf : explicit path overrides (win over world).
        node_->declare_parameter("manager/world", std::string("dokdo"));
        node_->declare_parameter("manager/esdf_dir", std::string("src/mmp_terrain/data"));
        node_->declare_parameter("manager/save_terrain_esdf", std::string());
        node_->declare_parameter("manager/load_terrain_esdf", std::string());

        std::string world_name;
        std::string esdf_dir;
        node_->get_parameter("manager/world", world_name);
        node_->get_parameter("manager/esdf_dir", esdf_dir);
        node_->get_parameter("manager/save_terrain_esdf", save_terrain_esdf_path_);
        node_->get_parameter("manager/load_terrain_esdf", load_terrain_esdf_path_);

        const std::string auto_path = esdf_dir + "/" + world_name + ".esdf";
        if (save_terrain_esdf_path_.empty()) save_terrain_esdf_path_ = auto_path;
        if (load_terrain_esdf_path_.empty()) load_terrain_esdf_path_ = auto_path;
        log_manager_->infof("ESDF cache for world='%s': load='%s' save='%s'",
                            world_name.c_str(),
                            load_terrain_esdf_path_.c_str(),
                            save_terrain_esdf_path_.c_str());

        // Parse risk zones: [cx, cy, cz, sensing_range, max_risk_level, ...]
        node_->declare_parameter("risk_zones", std::vector<double>{});
        std::vector<double> tz_params;
        node_->get_parameter("risk_zones", tz_params);
        log_manager_->infof("Risk zone params size: %zu", tz_params.size());
        if (tz_params.size() >= 5 && tz_params.size() % 5 == 0) {
            for (size_t ti = 0; ti < tz_params.size(); ti += 5) {
                RiskZone tz;
                tz.center = Eigen::Vector3d(tz_params[ti], tz_params[ti+1], tz_params[ti+2]);
                tz.reach = tz_params[ti+3];
                tz.peak = tz_params[ti+4];
                risk_zones_.push_back(tz);
                log_manager_->infof("  RiskZone #%zu: center=(%.1f,%.1f,%.1f) range=%.1f risk=%.1f",
                    risk_zones_.size()-1, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.reach, tz.peak);
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
            "/planning/front_end_path", 10);
        search_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/debug/search_path", 10);
        shorten_path_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/debug/shorten_path", 10);
        esdf_occ_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/debug/esdf_occupied", 1);
        // TRANSIENT_LOCAL so RViz, joining late, still gets the latest set.
        rclcpp::QoS dyn_qos(1);
        dyn_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        dyn_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        dyn_obstacle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/dynamic_obstacles", dyn_qos);

        // Terrain ESDF cache status (drone_0 only, latched).
        if (drone_id == 0) {
            rclcpp::QoS status_qos(1);
            status_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            terrain_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
                "/planning/terrain_status", status_qos);

            // Startup check: warn if load path is set but the cache file is missing.
            // path_manager builds the ESDF lazily on the first plan, so this is
            // a heads-up that the first plan will take a while.
            if (!load_terrain_esdf_path_.empty()) {
                std::ifstream f(load_terrain_esdf_path_);
                if (!f.good()) {
                    publishTerrainStatus(
                        "ESDF cache missing: " + load_terrain_esdf_path_ +
                        " — will build from TIF on first plan (this may take "
                        "several minutes; subsequent runs load instantly).");
                } else {
                    publishTerrainStatus("ESDF cache ready: " + load_terrain_esdf_path_);
                }
            }
        }
    }

    void PathManager::publishTerrainStatus(const std::string &msg) {
        if (!terrain_status_pub_) return;
        std_msgs::msg::String m;
        m.data = msg;
        terrain_status_pub_->publish(m);
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
                    oz.reach = tz.reach;
                    oz.peak = tz.peak;
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
            bound_margin_xy = std::max(bound_margin_xy, tz.reach + 5.0);
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
            double r = tz.reach + 5.0;
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
        // Upper bound: max(terrain peak, flight altitude) + headroom. The max()
        // keeps the flight altitude inside the box even over low terrain.
        map_upper_bound_.z() = std::max(max_terrain_z, map_upper_bound_.z()) + 20.0;

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
                publishTerrainStatus("ESDF loaded: " + load_terrain_esdf_path_);
            } else {
                log_manager_->warnf("SDF load failed from %s; falling back to build",
                                    load_terrain_esdf_path_.c_str());
            }
        }

        // Build SDF (fallback or no cache).
        if (!sdf_loaded_from_file_) {
            const bool will_persist = use_cache && !save_terrain_esdf_path_.empty();
            if (will_persist) {
                publishTerrainStatus(
                    "Building ESDF cache → " + save_terrain_esdf_path_ +
                    " (this may take several minutes; please wait)...");
            }
            if (!buildSDFForBounds(sdf_lo, sdf_hi)) {
                RCLCPP_ERROR(node_->get_logger(), "SDF build failed");
                if (will_persist) {
                    publishTerrainStatus("ESDF build failed (see path_manager log)");
                }
                return false;
            }
            // Persist on first successful build if requested (terrain must exist).
            if (will_persist) {
                if (sdf_manager_.saveToFile(save_terrain_esdf_path_)) {
                    log_manager_->infof("SDF saved to %s",
                                        save_terrain_esdf_path_.c_str());
                    sdf_loaded_from_file_ = true;  // skip rebuild next mission
                    publishTerrainStatus("ESDF cache ready: " + save_terrain_esdf_path_);
                } else {
                    publishTerrainStatus(
                        "ESDF build done but save failed → " + save_terrain_esdf_path_);
                }
            }
        }


        // The SDF is now ready — add any obstacles deferred before it existed
        // (no ESDF cache on a fresh run), so they make it into this first plan.
        flushPendingObstacles();

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


        // === STEP 2~3: front-end search + densification ===
        std::vector<Eigen::Vector3d> full_route, clean_path;
        if (!planFrontEnd(start_pos, waypoints, full_route, clean_path)) {
            return false;
        }

        // === STEP 4~5: trajectory optimization (MINCO + L-BFGS) ===
        bool opt_ok = optimizeStage(clean_path, full_route,
                                    start_pos, start_vel, start_acc, waypoints);

        auto t_total_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] === TOTAL planGlobalTraj: %.1f ms ===",
            std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count());

        return opt_ok;
    }

bool PathManager::planFrontEnd(const Eigen::Vector3d &start_pos,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               std::vector<Eigen::Vector3d> &full_route,
                               std::vector<Eigen::Vector3d> &clean_path)
{
        // Segment list: start -> wp1 -> ... -> wpN
        std::vector<Eigen::Vector3d> all_points;
        all_points.push_back(start_pos);
        for (const auto& wp : waypoints) {
            all_points.push_back(wp);
        }

        // === STEP 2: 3D A* search + visibility-thinning simple_path ===
        // Bind SDF + risk zones to the A* front-end. A* collision check uses
        // the ESDF (distance < obstacle_clearance_ == blocked), and risk
        // cost is added to the A* g-score per visited cell.
        std::vector<path_planner::search::RiskZoneLite> astar_risks;
        astar_risks.reserve(risk_zones_.size());
        for (const auto &tz : risk_zones_) {
            astar_risks.push_back({tz.center, tz.reach, tz.peak});
        }
        Eigen::Vector3d map_size = map_upper_bound_ - map_lower_bound_;
        searcher_.setLogManager(log_manager_);
        searcher_.setSDF(&sdf_manager_, map_lower_bound_, map_size, sdf_voxel_size_);
        const std::vector<path_planner::search::RiskZoneLite> *astar_tz_ptr =
            astar_risks.empty() ? nullptr : &astar_risks;
        searcher_.setRiskZones(astar_tz_ptr);
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
        searcher_.setObstacleMargin(obstacle_clearance_);
        searcher_.setSearchIgnoresObstacles(false);
        searcher_.setGroundHeight(ground_height_);
        searcher_.setVirtualCeilHeight(virtual_ceil_height_);
        searcher_.setRiskAlpha(risk_weight_);
        searcher_.setRiskBarrier(risk_barrier_);
        searcher_.setSmhaW(risk_smha_w_);
        searcher_.setFrontEnd(front_end_str_ == "fm2"
            ? path_planner::search::PathSearcher::FrontEnd::FM2
            : path_planner::search::PathSearcher::FrontEnd::ASTAR);
        searcher_.setFm2CoarseK(fm2_coarse_k_);
        searcher_.setFm2Star(fm2_star_);
        searcher_.setBypassShortcut(astar_bypass_shortcut_);

        // A* fine pool is only used by the A* front-end. FM2 runs on its
        // own coarse grid (fm2_F_, fm2_T_) and never touches pool_, so
        // skip the ~2 GB / 655 ms allocation in FM2 mode.
        if (front_end_str_ != "fm2") {
            Eigen::Vector3i sdf_shape = sdf_manager_.shape();
            if (sdf_shape.minCoeff() <= 0) {
                log_manager_->errorf("SDF shape not available, cannot size A* pool");
                return false;
            }
            Eigen::Vector3i desired = sdf_shape;
            if (!astar_initialized_) {
                astar_pool_size_ = desired;
                searcher_.initGridMap(astar_pool_size_);
                astar_initialized_ = true;
                log_manager_->infof("A* pool allocated: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            } else if (desired != astar_pool_size_) {
                astar_pool_size_ = desired;
                searcher_.resizePool(astar_pool_size_);
                log_manager_->infof("A* pool resized: (%d,%d,%d)",
                    astar_pool_size_.x(), astar_pool_size_.y(), astar_pool_size_.z());
            }
        }

        auto t_astar_start = std::chrono::steady_clock::now();
        full_route.clear();
        full_route.push_back(start_pos);
        for (size_t seg = 0; seg < all_points.size() - 1; ++seg)
        {
            std::vector<Eigen::Vector3d> seg_path =
                searcher_.astarSearchAndGetSimplePath(
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
        // DEBUG: annotate each A* waypoint with the SAME risk the planner uses,
        // i.e. the quadratic moat m_i = peak*(1 - d/reach)^2 (see dyn_a_star.h /
        // poly_traj_optimizer RiskGradCostP) and the OR-composed risk
        // = 1 - prod_i (1 - m_i), in [0,1]. (Previously this logged a Gaussian *
        // risk_weight, which did NOT match the planner and made edge passes look
        // far riskier than they are.)
        for (size_t ri = 0; ri < full_route.size(); ++ri) {
            const auto &p = full_route[ri];
            double survival = 1.0;
            std::string per_zone;
            for (size_t zi = 0; zi < risk_zones_.size(); ++zi) {
                const auto &tz = risk_zones_[zi];
                double dist = (p - tz.center).norm();
                double moat = 0.0;
                if (dist < tz.reach) {
                    double u = 1.0 - dist / tz.reach;
                    moat = tz.peak * u * u;
                }
                survival *= (1.0 - std::min(moat, 1.0 - 1e-3));
                char buf[64];
                std::snprintf(buf, sizeof(buf), " tz%zu(d=%.2f,m=%.3f)",
                              zi, dist, moat);
                per_zone += buf;
            }
            log_manager_->infof("  A*[%zu]: (%.2f, %.2f, %.2f) risk=%.3f%s",
                ri, p.x(), p.y(), p.z(), 1.0 - survival, per_zone.c_str());
        }

        // Front-end route as nav_msgs/Path → mmp_visualization converts it to a
        // RViz marker (/viz/simple_path).
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

        // Front-end route as LINE_STRIP + SPHERE_LIST for debugging (cyan)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            line.ns = "search_path_line";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 1.5;
            line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; line.color.a = 1.0;
            line.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                line.points.push_back(pt);
            }
            search_path_pub_->publish(line);

            visualization_msgs::msg::Marker dots;
            dots.header = line.header;
            dots.ns = "search_path_dots";
            dots.id = 1;
            dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            dots.action = visualization_msgs::msg::Marker::ADD;
            dots.pose.orientation.w = 1.0;
            dots.scale.x = 2.5; dots.scale.y = 2.5; dots.scale.z = 2.5;
            dots.color.r = 0.0; dots.color.g = 0.8; dots.color.b = 1.0; dots.color.a = 1.0;
            dots.lifetime = rclcpp::Duration(0, 0);
            for (const auto &p : full_route) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
                dots.points.push_back(pt);
            }
            search_path_pub_->publish(dots);
        }

        // === STEP 3: Corner-adaptive densification of the A* shortcut. ===
        // Two spacing regimes: dense near sharp corners (small kinks absorb the
        // turn so the 5th-order MINCO polynomial can't loop), coarse on long
        // straights (big loosely-linked pieces L-BFGS can reshape freely).
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

        clean_path.clear();
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
            line.scale.x = 1.5;
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

        return true;
}

bool PathManager::optimizeStage(std::vector<Eigen::Vector3d> &clean_path,
                                const std::vector<Eigen::Vector3d> &full_route,
                                const Eigen::Vector3d &start_pos,
                                const Eigen::Vector3d &start_vel,
                                const Eigen::Vector3d &start_acc,
                                const std::vector<Eigen::Vector3d> &waypoints)
{
        // Stage 2 = trajectory optimization. The optimizer owns the MINCO
        // initial-trajectory build + L-BFGS; we only pass the front-end path
        // and store the results. Swap optimizeFromPath() to replace the backend.
        if (!isOptimizerInitialized()) {
            log_manager_->errorf("Optimizer not initialized");
            return false;
        }

        auto t_opt_start = std::chrono::steady_clock::now();

        double global_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        poly_traj::Trajectory global_traj, local_traj;
        bool opt_success = poly_traj_opt_->optimizeFromPath(
            clean_path, start_pos, start_vel, start_acc, waypoints, max_vel_,
            global_traj, local_traj);
        if (!opt_success) {
            log_manager_->errorf("Trajectory optimization failed");
            return false;
        }

        // Local traj start_time is the trajectory-following reference; stamp it
        // after optimization (matches pre-refactor behavior).
        double local_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        traj_.setGlobalTraj(global_traj, global_time);
        traj_.setLocalTraj(local_traj, local_time, traj_.local_traj.drone_id);
        simple_path_ = full_route;

        auto t_opt_end = std::chrono::steady_clock::now();
        log_manager_->infof("[TIMING] trajectory optimization: %.1f ms, duration=%.3f max_vel=%.3f",
            std::chrono::duration<double, std::milli>(t_opt_end - t_opt_start).count(),
            local_traj.getTotalDuration(), local_traj.getMaxVelRate());

        return true;
}

bool PathManager::isMapReady(const Eigen::Vector3d& /*start_pos*/) const {
    // Phase 4: obstacles are always loaded from yaml, so map is always ready.
    return true;
}

void PathManager::setFormationInfo(int drone_id, const std::string& formation_type,
                                   const std::vector<Eigen::Vector3d>& formation_pattern) {
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
            if (!sdf_manager_.isInitialized()) sdf_manager_.initialize(sdf_voxel_size_);
            if (sdf_manager_.loadFromFile(load_terrain_esdf_path_, lo, hi)) {
                sdf_loaded_from_file_ = true;
                log_manager_->infof("SDF eagerly loaded from %s",
                                    load_terrain_esdf_path_.c_str());
                flushPendingObstacles();
            }
        }
    }
}

const PathManager::ObstacleMeshInfo& PathManager::meshFor(const std::string& model) const
{
    auto it = mesh_catalog_.find(model.empty() ? std::string("building") : model);
    if (it == mesh_catalog_.end()) it = mesh_catalog_.find("building");
    return it->second;
}

int PathManager::addDynamicSphere(const Eigen::Vector3d& center, double radius,
                                  const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer: the SDF (ESDF) is built lazily on the first plan. Without a
        // cache it is not ready yet, so queue and add once it exists.
        pending_obstacles_.push_back({false, center,
            Eigen::Vector3d(radius, radius, radius), radius, model});
        log_manager_->infof("addDynamicSphere: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f r=%.2f)",
                            center.x(), center.y(), center.z(), radius);
        return -2;  // deferred (not an error)
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
    dyn_patch_sizes_.push_back(Eigen::Vector3d(d, d, d));  // sphere stored as (2r,2r,2r)
    dyn_patch_is_box_.push_back(0);
    dyn_patch_models_.push_back(model);
    dyn_patch_yaws_.push_back(
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_));
    log_manager_->infof("Dynamic sphere added: id=%d center=(%.2f,%.2f,%.2f) r=%.2f, total=%zu",
                        id, center.x(), center.y(), center.z(), radius,
                        sdf_manager_.numActiveObstacles());
    publishDynamicObstacles();
    return id;
}

int PathManager::addDynamicBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                               const std::string& model)
{
    if (!sdf_manager_.hasData()) {
        // Defer until the SDF is built (see addDynamicSphere).
        pending_obstacles_.push_back({true, center, size, 0.0, model});
        log_manager_->infof("addDynamicBox: SDF not ready, deferred "
                            "(center=%.2f,%.2f,%.2f)",
                            center.x(), center.y(), center.z());
        return -2;  // deferred (not an error)
    }
    path_planner::sdf::PrimitiveSpec spec;
    spec.kind = path_planner::sdf::PrimitiveKind::kCube;
    spec.center = center;
    spec.size = size;  // full extents (sx, sy, sz)

    int id = sdf_manager_.addObstacle(spec);
    if (id < 0) {
        log_manager_->warnf("addDynamicBox: addObstacle failed "
                            "(center=%.2f,%.2f,%.2f size=%.2f,%.2f,%.2f)",
                            center.x(), center.y(), center.z(),
                            size.x(), size.y(), size.z());
        return -1;
    }
    dyn_patch_ids_.push_back(id);
    dyn_patch_centers_.push_back(center);
    dyn_patch_sizes_.push_back(size);
    dyn_patch_is_box_.push_back(1);
    dyn_patch_models_.push_back(model);
    dyn_patch_yaws_.push_back(
        std::uniform_real_distribution<double>(0.0, 6.283185307179586)(yaw_rng_));
    log_manager_->infof("Dynamic box added: id=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f), total=%zu",
                        id, center.x(), center.y(), center.z(),
                        size.x(), size.y(), size.z(),
                        sdf_manager_.numActiveObstacles());
    publishDynamicObstacles();
    return id;
}

void PathManager::clearDynamicObstacles()
{
    sdf_manager_.clearObstacles();
    dyn_patch_ids_.clear();
    dyn_patch_centers_.clear();
    dyn_patch_sizes_.clear();
    dyn_patch_is_box_.clear();
    dyn_patch_models_.clear();
    dyn_patch_yaws_.clear();
    pending_obstacles_.clear();
    log_manager_->infof("Dynamic obstacles cleared");
    publishDynamicObstacles();
}

// Add obstacles that were requested before the SDF existed. Called right after
// the SDF is built/loaded, so they make it into the very first plan.
void PathManager::flushPendingObstacles()
{
    if (pending_obstacles_.empty() || !sdf_manager_.hasData()) return;
    std::vector<PendingObstacle> pend;
    pend.swap(pending_obstacles_);  // addDynamic* see an empty queue -> no re-defer
    for (const auto& p : pend) {
        if (p.is_box) addDynamicBox(p.center, p.size, p.model);
        else          addDynamicSphere(p.center, p.radius, p.model);
    }
    log_manager_->infof("Flushed %zu deferred dynamic obstacle(s) after SDF ready",
                        pend.size());
}

void PathManager::setRiskZonesRuntime(const std::vector<RiskZone>& zones)
{
    // Atomically replace the active zone list. The next planGlobalTraj
    // call will re-bind A* and the optimizer with the new set via the
    // existing setup paths (see planGlobalTraj where searcher_.setRiskZones
    // and poly_traj_opt_->setRiskZones are called).
    risk_zones_ = zones;
    if (log_manager_) {
        log_manager_->infof(
            "[risk_zones] runtime update: %zu zones now active",
            risk_zones_.size());
        for (size_t i = 0; i < risk_zones_.size(); ++i) {
            const auto& tz = risk_zones_[i];
            log_manager_->infof(
                "  zone[%zu] center=(%.1f,%.1f,%.1f) reach=%.1f peak=%.2f",
                i, tz.center.x(), tz.center.y(), tz.center.z(),
                tz.reach, tz.peak);
        }
    }
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
        // Per-obstacle visual model from the catalog; collision (SDF) is separate.
        const ObstacleMeshInfo& mi = meshFor(dyn_patch_models_[i]);
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "dynamic_obstacles";
        m.id = dyn_patch_ids_[i];
        // color all-zero => RViz uses the mesh's own embedded materials/textures.
        m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        m.mesh_resource = mi.resource;
        m.mesh_use_embedded_materials = true;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.color.r = m.color.g = m.color.b = m.color.a = 0.0f;  // use mesh textures

        const Eigen::Vector3d& c = dyn_patch_centers_[i];
        double foot_x, foot_y, height, base_z;
        if (dyn_patch_is_box_[i]) {
            // Box: mesh fills the collision box exactly (collision == visual bbox).
            foot_x = dyn_patch_sizes_[i].x();
            foot_y = dyn_patch_sizes_[i].y();
            height = dyn_patch_sizes_[i].z();
            base_z = c.z() - 0.5 * height;
        } else {
            // Sphere: fixed-height building from the ball's bottom.
            const double radius = 0.5 * dyn_patch_sizes_[i].x();  // stored as (2r,2r,2r)
            foot_x = foot_y = 2.0 * radius;
            height = obstacle_mesh_height_;
            base_z = c.z() - radius;
        }

        m.pose.position.x = c.x();
        m.pose.position.y = c.y();
        m.pose.position.z = base_z;
        // Per-spawn random yaw about Z (visual only; SDF collision stays AABB).
        const double yaw = (i < dyn_patch_yaws_.size()) ? dyn_patch_yaws_[i] : 0.0;
        m.pose.orientation.z = std::sin(0.5 * yaw);
        m.pose.orientation.w = std::cos(0.5 * yaw);
        m.scale.x = foot_x / mi.native_size.x();
        m.scale.y = foot_y / mi.native_size.y();
        m.scale.z = height / mi.native_size.z();
        arr.markers.push_back(m);
    }
    dyn_obstacle_pub_->publish(arr);
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
