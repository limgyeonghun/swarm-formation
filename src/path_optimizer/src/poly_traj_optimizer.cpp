#include "path_optimizer/poly_traj_optimizer.h"
#include <iomanip>
#include <ctime>
#include <sys/resource.h>

namespace ego_planner
{
  void PolyTrajOptimizer::setLogManager(swarm_formation::LogManager::Ptr log_manager)
  {
    log_manager_ = log_manager;
  }

  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, const bool use_formation)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      return false;
    }

    t_now_ = node_->get_clock()->now().seconds();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);
    Eigen::Vector3d start_pos = iniState.col(0);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    auto t0 = node_->get_clock()->now();
    auto t1 = node_->get_clock()->now();
    auto t2 = node_->get_clock()->now();
    bool use_formation_temp = use_formation_;

    std::vector<double> q(variable_num_);
    memcpy(q.data(), initInnerPts.data(), initInnerPts.size() * sizeof(double));
    Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    // 3. L-BFGS parameter setup
    auto t3 = node_->get_clock()->now();
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;

    if (use_formation)
    {
      lbfgs_params.max_iterations = 20;
    }
    else
    {
      lbfgs_params.max_iterations = 60;
      use_formation_ = false;
    }

    // Debug: Print L-BFGS parameters
    if (log_manager_ && enable_debug_logs_) {
        log_manager_->infof("L-BFGS params: mem_size=%d, max_iter=%d, g_epsilon=%f, min_step=%f, use_formation=%d",
          lbfgs_params.mem_size, lbfgs_params.max_iterations, lbfgs_params.g_epsilon, 
          lbfgs_params.min_step, use_formation);
    }

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    t1 = node_->get_clock()->now();

    int result = lbfgs::lbfgs_optimize(
        variable_num_,
        q.data(),
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);
    // Log L-BFGS result (only for debugging)
    if (log_manager_ && enable_debug_logs_) {
        const char* result_str = lbfgs::lbfgs_strerror(result);
        log_manager_->infof("L-BFGS Result: %d (%s)", result, result_str);
        log_manager_->infof("Iteration info: costFunction calls=%d, max_iterations=%d", iter_num_, lbfgs_params.max_iterations);
    }

    // Collision check (only if obstacles are enabled)
    bool occ = enable_obstacles_ ? checkCollision() : false;

    use_formation_ = use_formation_temp;

    t2 = node_->get_clock()->now();
    double time_ms = (t2 - t1).seconds() * 1000;
    double total_time_ms = (t2 - t0).seconds() * 1000;

    // Final result logging
    if (log_manager_) {
        log_manager_->infof("Optimization completed: iter=%d, use_formation=%d, time(ms)=%f", iter_num_, use_formation, time_ms);

        // Calculate and log jerk metrics
        poly_traj::Trajectory final_traj = jerkOpt_.getTraj();
        double total_jerk = computeTotalJerk(final_traj);
        double max_jerk = computeMaxJerk(final_traj);
        log_manager_->infof("[JERK METRICS] total_jerk=%.6f, max_jerk=%.6f m/s³, duration=%.3f s",
                            total_jerk, max_jerk, final_traj.getTotalDuration());

        log_manager_->infof("[COST] formation_cost=%f (wei_formation=%f, similarity=%f)", dbg_cost_formation_, wei_formation_, debug_similarity_);

        // Nonholonomic constraint summary
        // ALWAYS log for Before/After comparison (even when weight=0 and no violations)
        double total_nonholo_cost = dbg_cost_curvature_ + dbg_cost_braking_ +
                                     dbg_cost_fwd_vel_ + dbg_cost_lat_accel_;
        int total_violations = dbg_curv_violations_ + dbg_brake_violations_ +
                               dbg_fwd_vel_violations_ + dbg_lat_accel_violations_;

        // Always log (unconditional) to ensure consistent Before/After data
        log_manager_->infof("[NONHOLONOMIC SUMMARY] total_violations=%d, total_cost=%.6f (weight=%.3f)",
                             total_violations, total_nonholo_cost, wei_nonholo_);

        if (total_violations > 0 || wei_nonholo_ > 0.0) {
            if (dbg_curv_violations_ > 0) {
                log_manager_->infof("  ⚠ Curvature violations: %d (max_κ=%.4f > limit=%.4f)",
                                     dbg_curv_violations_, dbg_max_curvature_, max_curvature_);
            }
            if (dbg_brake_violations_ > 0) {
                log_manager_->infof("  ⚠ Braking violations: %d (min_decel=%.4f < limit=-%.4f m/s²)",
                                     dbg_brake_violations_, dbg_max_brake_decel_, max_brake_decel_);
            }
            if (total_violations == 0 && wei_nonholo_ > 0.0) {
                log_manager_->infof("  ✓ All nonholonomic constraints satisfied! max_κ=%.4f (limit=%.4f)",
                                     dbg_max_curvature_, max_curvature_);
            }
        }

        // Additional debugging info
        if (enable_debug_logs_) {
            const char* result_str = lbfgs::lbfgs_strerror(result);
            log_manager_->debugf("L-BFGS Final Result: %d (%s)", result, result_str);
            log_manager_->debugf("Final iteration info: costFunction calls=%d, max_iterations=%d, Final cost=%f",
              iter_num_, lbfgs_params.max_iterations, final_cost);
        }
    } else {
        // Fallback to macro logger if log_manager is not available
        LOG_INFO("id=%d, iter=%d, use_formation=%d, time(ms)=%.3f",
                 drone_id_, iter_num_, use_formation, time_ms);
        LOG_INFO("[COST] formation_cost=%.6f (wei_formation=%.3f, similarity=%.6f)",
                 dbg_cost_formation_, wei_formation_, debug_similarity_);
    }
    optimal_points = cps_.points;

    showFormationInformation(false, start_pos);

    if (occ)
      return false;
    else
      return true;
  }
  bool PolyTrajOptimizer::checkCollision(void)
  {
    double T_end;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
  
    int N = traj.getPieceNum();
    int k = cps_num_prePiece_ * N + 1;
    int idx = k / 3 * 2;
    int piece_of_idx = floor((idx - 1) / cps_num_prePiece_);
    Eigen::VectorXd durations = traj.getDurations();

    if (piece_of_idx < 0 || piece_of_idx >= N) {
      T_end = durations.sum();
    } else {
      T_end = durations.head(piece_of_idx).sum()
            + durations(piece_of_idx)
            * (idx - piece_of_idx * cps_num_prePiece_) / (double)cps_num_prePiece_;
    }
  
    bool occ = false;
    double dt = 0.01;
    int i_end = std::max(1, (int)floor(T_end / dt));
    double t = 0.0;
    collision_check_time_end_ = T_end;
  
    for (int i = 0; i < i_end; i++)
    {
      Eigen::Vector3d pos = traj.getPos(t);

      int infl = grid_map_->getInflateOccupancy(pos);
      if (infl == 1)
      {
        bool in_map   = grid_map_->isInMap(pos);
        bool in_road  = grid_map_->isInRoadBoundary(pos);
        int  occ_raw  = grid_map_->getOccupancy(pos);
        double dist_esdf = 0.0;
        grid_map_->evaluateEDT(pos, dist_esdf);
  
        LOG_WARN("[COLLISION] t=%.3f pos=(%.3f, %.3f, %.3f) infl=1 in_map=%d in_road=%d occ=%d esdf=%.3f",
                 t, pos.x(), pos.y(), pos.z(), (int)in_map, (int)in_road, occ_raw, dist_esdf);

        if (in_map && !in_road) {
          LOG_WARN("[COLLISION] Road-boundary violation at t=%.3f (treated as obstacle). "
                   "Check grid_map.use_road_boundary/road_width/road_margin/segments.", t);
        }

        if (!in_map) {
          LOG_WARN("[COLLISION] Out of map at t=%.3f. Check map size/origin/resolution.", t);
        }
  
        occ = true;
        break;
      }
  
      t += dt;
    }
    return occ;
  }

  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(6);

    // High-performance timing for debugging (similar to con code)
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t1 = t_start, t2 = t_start, t3 = t_start, t4 = t_start, t5 = t_start;

    // 1. Trajectory generation
    t1 = std::chrono::high_resolution_clock::now();
    opt->jerkOpt_.generate(P, T);
    double traj_gen_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t1).count();

    // 2. Smoothness cost
    t2 = std::chrono::high_resolution_clock::now();
    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost
    double smoothness_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t2).count();

    // 3. Obstacle/Swarm/Feasibility cost (most complex part)
    t3 = std::chrono::high_resolution_clock::now();
    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost
    double pva_cost_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t3).count();

    // 4. Gradient calculation
    t4 = std::chrono::high_resolution_clock::now();
    opt->jerkOpt_.getGrad2TP(gradT, gradP);
    double grad_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t4).count();

    // 5. Time cost
    t5 = std::chrono::high_resolution_clock::now();
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);
    double time_cost_time = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t5).count();

    opt->iter_num_ += 1;

    // Debug output for performance monitoring
    if (opt->iter_num_ % 50 == 0 && opt->log_manager_ && opt->enable_debug_logs_) {
        double total_callback_time = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        opt->log_manager_->debugf("CostFunction iter=%d: Traj=%fms, Smooth=%fms, PVA=%fms, Grad=%fms, Time=%fms, Total=%fms", 
          opt->iter_num_, traj_gen_time, smoothness_time, pva_cost_time, grad_time, time_cost_time, total_callback_time);
    }

    // Detailed L-BFGS cost debugging
    if (opt->enable_lbfgs_detail_logs_ && opt->log_manager_ && opt->iter_num_ % 10 == 0) {
        double total_cost = smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
        opt->log_manager_->infof("[L-BFGS DETAIL] iter=%d, total_cost=%.6f", opt->iter_num_, total_cost);
        opt->log_manager_->infof("  smoothness_cost=%.6f (weight=implicit)", smoo_cost);
        opt->log_manager_->infof("  obstacle_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(0), opt->wei_obs_);
        opt->log_manager_->infof("  swarm_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(1), opt->wei_swarm_);
        opt->log_manager_->infof("  formation_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(2), opt->wei_formation_);
        opt->log_manager_->infof("  feasibility_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(4), opt->wei_feas_);
        opt->log_manager_->infof("  time_cost=%.6f (weight=%.3f)", time_cost, opt->wei_time_);

        // Detailed nonholonomic constraint violation logging
        // ALWAYS log if violations exist (for Before/After comparison analysis)
        double total_nonholo_cost = opt->dbg_cost_curvature_ + opt->dbg_cost_braking_ +
                                     opt->dbg_cost_fwd_vel_ + opt->dbg_cost_lat_accel_;
        int total_violations = opt->dbg_curv_violations_ + opt->dbg_brake_violations_ +
                               opt->dbg_fwd_vel_violations_ + opt->dbg_lat_accel_violations_;

        // Log if: (1) violations exist, OR (2) weight > 0 (to show constraint is active)
        if (total_violations > 0 || opt->wei_nonholo_ > 0.0) {
            opt->log_manager_->infof("[NONHOLONOMIC VIOLATIONS] total_cost=%.6f (weight=%.3f)",
                                      total_nonholo_cost, opt->wei_nonholo_);
            opt->log_manager_->infof("  curvature: violations=%d, cost=%.6f, max_κ=%.4f (limit=%.4f)",
                                      opt->dbg_curv_violations_, opt->dbg_cost_curvature_,
                                      opt->dbg_max_curvature_, opt->max_curvature_);
            opt->log_manager_->infof("  braking: violations=%d, cost=%.6f, min_decel=%.4f m/s² (limit=-%.4f)",
                                      opt->dbg_brake_violations_, opt->dbg_cost_braking_,
                                      opt->dbg_max_brake_decel_, opt->max_brake_decel_);
            opt->log_manager_->infof("  fwd_velocity: violations=%d, cost=%.6f (min_vel=%.4f m/s)",
                                      opt->dbg_fwd_vel_violations_, opt->dbg_cost_fwd_vel_,
                                      opt->min_forward_vel_);
            opt->log_manager_->infof("  lateral_accel: violations=%d, cost=%.6f (limit=%.4f m/s²)",
                                      opt->dbg_lat_accel_violations_, opt->dbg_cost_lat_accel_,
                                      opt->max_lateral_accel_);
        }

        // Store formation cost for final logging
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    } else if (!opt->enable_lbfgs_detail_logs_ && opt->use_formation_) {
        // Store formation cost for final logging even when detail logs are disabled
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    }

    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx,
                                           const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      if (VT(i) > 0.0) {
        RT(i) = ((0.5 * VT(i) + 1.0) * VT(i) + 1.0);
      } else {
        double denom = ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
        // Protect against near-zero denominator
        if (std::abs(denom) < 1e-10) {
          if (log_manager_) {
            log_manager_->errorf("VirtualT2RealT: Near-zero denominator at i=%d, VT=%f, denom=%e",
                                i, VT(i), denom);
          }
          RT(i) = 1.0;  // Fallback to minimum time
        } else {
          RT(i) = 1.0 / denom;
        }
      }
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }
      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }
    costT = RT.sum() * wei_time_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {

    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0;

    // Reset nonholonomic constraint violation tracking
    dbg_curv_violations_ = 0;
    dbg_brake_violations_ = 0;
    dbg_fwd_vel_violations_ = 0;
    dbg_lat_accel_violations_ = 0;
    dbg_max_curvature_ = 0.0;
    dbg_max_brake_decel_ = 0.0;
    dbg_cost_curvature_ = 0.0;
    dbg_cost_braking_ = 0.0;
    dbg_cost_fwd_vel_ = 0.0;
    dbg_cost_lat_accel_ = 0.0;

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // Obstacle cost calculation
        if (enable_obstacles_) {
            if (obstacleGradCostP(i_dp, pos, gradp, costp)) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradp.transpose() * vel;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                costs(0) += omg * step * costp;
            }
        }

        double gradt, grad_prev_t;

        // Swarm collision cost calculation - now computed for every point for maximum accuracy
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)) {
            gradViolaPc = beta0 * gradp.transpose();
            gradViolaPt = alpha * gradt;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
            gdT(i) += omg * (costp / K + step * gradViolaPt);
            if (i > 0) {
                gdT.head(i).array() += omg * step * grad_prev_t;
            }
            costs(1) += omg * step * costp;
        }

        // Formation cost calculation - now computed for every point for maximum accuracy
        if (use_formation_) {
            if (swarmGraphGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)) {
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0) {
                    gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(2) += omg * step * costp;
            }
        }

        // Feasibility cost calculation
        if (feasibilityGradCostV(vel, gradv, costv)) {
            gradViolaVc = beta1 * gradv.transpose();
            gradViolaVt = alpha * gradv.transpose() * acc;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
            gdT(i) += omg * (costv / K + step * gradViolaVt);
            costs(4) += omg * step * costv;
        }
        if (feasibilityGradCostA(acc, grada, costa)) {
            gradViolaAc = beta2 * grada.transpose();
            gradViolaAt = alpha * grada.transpose() * jer;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
            gdT(i) += omg * (costa / K + step * gradViolaAt);
            costs(4) += omg * step * costa;
        }

        // Nonholonomic constraint cost (forward velocity + braking + curvature + lateral acceleration)
        Eigen::Vector3d grad_nonholo_vel, grad_nonholo_acc;
        double cost_nonholo;
        if (nonholonomicGradCost(vel, acc, grad_nonholo_vel, grad_nonholo_acc, cost_nonholo)) {
            // Gradient w.r.t. control points from velocity
            gradViolaVc = beta1 * grad_nonholo_vel.transpose();
            gradViolaVt = alpha * grad_nonholo_vel.transpose() * acc;

            // Gradient w.r.t. control points from acceleration
            gradViolaAc = beta2 * grad_nonholo_acc.transpose();
            gradViolaAt = alpha * grad_nonholo_acc.transpose() * jer;

            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * (gradViolaVc + gradViolaAc);
            gdT(i) += omg * (cost_nonholo / K + step * (gradViolaVt + gradViolaAt));
            costs(4) += omg * step * cost_nonholo;  // Add to feasibility cost
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
            ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i) {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      for (int j = 0; j <= K; ++j) {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;
        omg = (j == 0 || j == K) ? 0.5 : 1.0;
        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);
        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
            ++i_dp;
        }
      }
    }
    costs(5) += var;

    dbg_cost_formation_ = costs(2);

  }

  bool PolyTrajOptimizer::swarmGraphGradCostP(const int i_dp,
                                              const double t,
                                              const Eigen::Vector3d &p,
                                              const Eigen::Vector3d &v,
                                              Eigen::Vector3d &gradp,
                                              double &gradt,
                                              double &grad_prev_t,
                                              double &costp)
  {
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    if (!swarm_trajs_) {
      return false;
    }

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
      return false;

    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    double pt_time = t_now_ + t;
    std::vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (size_t id = 0; id < size; id++)
    {
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    swarm_graph_->updateGraph(swarm_graph_pos);

    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);

    debug_similarity_ = similarity_error;

    if (similarity_error > 0)
    {
      ret = true;

      costp = wei_formation_ * similarity_error;
      std::vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);

      gradp = wei_formation_ * swarm_grad[drone_id_];

      for (size_t id = 0; id < size; id++)
      {
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {

    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool ret = false;
    gradp.setZero();
    costp = 0;

    double dist;
    grid_map_->evaluateEDT(p, dist);
    double dist_err = obs_clearance_ - dist;
    if (dist_err > 0)
    {
      ret = true;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateFirstGrad(p, dist_grad);

      costp = wei_obs_ * pow(dist_err, 3);
      gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // Check for nullptr before accessing swarm_trajs_
    if (!swarm_trajs_) {
      return false;
    }

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }

      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 +
                            (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;
        costp += wei_swarm_ * dist2_err3;
        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) *
                                    Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
      }
    }
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::nonholonomicGradCost(const Eigen::Vector3d &vel,
                                               const Eigen::Vector3d &acc,
                                               Eigen::Vector3d &grad_vel,
                                               Eigen::Vector3d &grad_acc,
                                               double &cost_nonholo)
  {
    /**
     * Nonholonomic Constraints for Rover Dynamics
     *
     * Combines 4 constraints to prevent backward motion, sharp turns, and sudden braking:
     * 1. Forward velocity constraint: v_forward > v_min (prevent backward/hovering)
     * 2. Braking deceleration constraint: a_forward > -a_brake_max (prevent sudden stops)
     * 3. Curvature constraint: κ < κ_max (prevent sharp turns)
     * 4. Centripetal acceleration: v²×κ < a_lat_max (speed-curvature coupling)
     */

    grad_vel.setZero();
    grad_acc.setZero();
    cost_nonholo = 0.0;

    double cost1 = 0.0, cost2 = 0.0, cost3 = 0.0, cost4 = 0.0;  // Individual costs
    double v_norm = vel.norm();

    // Skip if velocity is too small (stationary or near-stationary)
    if (v_norm < 1e-4) {
      // Penalize zero velocity (hovering not allowed for rover)
      if (min_forward_vel_ > 1e-3) {
        double vel_deficit = min_forward_vel_;
        cost_nonholo = wei_nonholo_ * vel_deficit * vel_deficit * vel_deficit;
        // Gradient is zero since velocity is already zero
      }
      return cost_nonholo > 0.0;
    }

    // Calculate heading (velocity direction)
    Eigen::Vector3d heading = vel / v_norm;

    // ========== Constraint 1: Minimum Forward Velocity ==========
    // Prevent backward motion and hovering
    double v_forward = v_norm;  // In 2D/3D, we want minimum speed

    if (v_forward < min_forward_vel_) {
      double fwd_deficit = min_forward_vel_ - v_forward;
      cost1 = wei_nonholo_ * fwd_deficit * fwd_deficit * fwd_deficit;

      // Gradient: ∂cost/∂v = ∂cost/∂||v|| · ∂||v||/∂v
      Eigen::Vector3d grad1_v = wei_nonholo_ * 3.0 * fwd_deficit * fwd_deficit * (-heading);

      cost_nonholo += cost1;
      grad_vel += grad1_v;
      dbg_fwd_vel_violations_++;
      dbg_cost_fwd_vel_ += cost1;
    }

    // ========== Constraint 2: Maximum Braking Deceleration ==========
    // Prevent sudden stops (deceleration in direction of motion)
    double a_forward = acc.dot(heading);

    if (a_forward < -max_brake_decel_) {
      double brake_excess = -max_brake_decel_ - a_forward;
      cost2 = wei_nonholo_ * brake_excess * brake_excess * brake_excess;

      // Gradient w.r.t. acceleration
      Eigen::Vector3d grad2_a = wei_nonholo_ * 3.0 * brake_excess * brake_excess * (-heading);

      // Gradient w.r.t. velocity (from heading dependency)
      // ∂(a·h)/∂v = ∂(a·(v/||v||))/∂v = (a - (a·h)h) / ||v||
      Eigen::Vector3d grad2_v = wei_nonholo_ * 3.0 * brake_excess * brake_excess *
                                (acc - a_forward * heading) / v_norm;

      cost_nonholo += cost2;
      grad_acc += grad2_a;
      grad_vel += grad2_v;
      dbg_brake_violations_++;
      dbg_cost_braking_ += cost2;
      if (a_forward < dbg_max_brake_decel_) {
        dbg_max_brake_decel_ = a_forward;
      }
    }

    // ========== Constraint 3: Curvature Constraint ==========
    // Prevent sharp turns (minimum turning radius)
    Eigen::Vector3d v_cross_a = vel.cross(acc);
    double cross_norm = v_cross_a.norm();

    // Use a minimum velocity threshold to prevent numerical instability
    // When v is very small, curvature becomes unreliable, so we clip it
    const double v_min_for_curvature = 0.05;  // 5 cm/s minimum
    double v_safe = std::max(v_norm, v_min_for_curvature);
    double v_safe3 = v_safe * v_safe * v_safe;

    // Calculate curvature once and reuse for both constraints
    double curvature = 0.0;
    Eigen::Vector3d dcurv_dcross = Eigen::Vector3d::Zero();
    double dcurv_dvnorm = 0.0;

    if (cross_norm > 1e-6) {
      // Use safe velocity for curvature calculation (prevents division by near-zero)
      curvature = cross_norm / v_safe3;

      // Track maximum curvature
      if (curvature > dbg_max_curvature_) {
        dbg_max_curvature_ = curvature;
      }

      // Pre-compute curvature gradients for reuse
      // ∂κ/∂(v×a) = 1 / v_safe³ · (v×a) / ||v×a||
      dcurv_dcross = v_cross_a / (cross_norm * v_safe3);
      // ∂κ/∂||v|| = -3κ / v_safe (only if v_norm >= v_min, else gradient is zero)
      dcurv_dvnorm = (v_norm >= v_min_for_curvature) ? (-3.0 * curvature / v_safe) : 0.0;

      if (curvature > max_curvature_) {
        double curv_excess = curvature - max_curvature_;
        cost3 = wei_nonholo_ * curv_excess * curv_excess * curv_excess;

        // ∂cost/∂κ
        double dcost_dcurv = wei_nonholo_ * 3.0 * curv_excess * curv_excess;

        // Gradient w.r.t. velocity
        Eigen::Vector3d grad3_v_cross = acc.cross(dcurv_dcross);
        Eigen::Vector3d grad3_v_norm = dcurv_dvnorm * heading;
        Eigen::Vector3d grad3_v = dcost_dcurv * (grad3_v_cross + grad3_v_norm);

        // Gradient w.r.t. acceleration
        Eigen::Vector3d grad3_a = dcost_dcurv * vel.cross(dcurv_dcross);

        cost_nonholo += cost3;
        grad_vel += grad3_v;
        grad_acc += grad3_a;
        dbg_curv_violations_++;
        dbg_cost_curvature_ += cost3;
      }
    }

    // ========== Constraint 4: Centripetal Acceleration ==========
    // Speed-curvature coupling: prevent rollover/slip
    if (cross_norm > 1e-6) {
      // Reuse curvature calculated above (no redundant computation)
      double centripetal_acc = v_norm * v_norm * curvature;

      if (centripetal_acc > max_lateral_accel_) {
        double lat_excess = centripetal_acc - max_lateral_accel_;
        cost4 = wei_nonholo_ * lat_excess * lat_excess * lat_excess;

        // ∂cost/∂(v²κ)
        double dcost_dlat = wei_nonholo_ * 3.0 * lat_excess * lat_excess;

        // Complete gradient: ∂(v²κ)/∂v = 2v·κ·heading + v²·∂κ/∂v·heading
        // ∂(v²κ)/∂(v×a) = v²·∂κ/∂(v×a)
        Eigen::Vector3d grad4_v_norm = 2.0 * v_norm * curvature * heading;
        Eigen::Vector3d grad4_v_curv = v_norm * v_norm * dcurv_dvnorm * heading;
        Eigen::Vector3d grad4_v = dcost_dlat * (grad4_v_norm + grad4_v_curv);

        Eigen::Vector3d grad4_v_cross = v_norm * v_norm * acc.cross(dcurv_dcross);
        Eigen::Vector3d grad4_a_cross = v_norm * v_norm * vel.cross(dcurv_dcross);
        Eigen::Vector3d grad4_a = dcost_dlat * grad4_a_cross;
        grad_vel += grad4_v + dcost_dlat * grad4_v_cross;
        grad_acc += grad4_a;

        cost_nonholo += cost4;
        dbg_lat_accel_violations_++;
        dbg_cost_lat_accel_ += cost4;
      }
    }

    return cost_nonholo > 0.0;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();

    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
  }

  void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           vector<Eigen::Vector3d> &simple_path,
                                           Eigen::MatrixXd &ctl_points,
                                           poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos = finState.col(0);

    simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos, drone_id_);

    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;
    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }
    frontendMJ.reset(iniState, finState, piece_num);

    double des_vel = max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      for (size_t i = 1; i <= piece_num; ++i)
      {
        time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                   : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
      }
      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.5;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ && debug_num < 1);

    poly_traj::Trajectory traj = frontendMJ.getTraj();
    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
  }

  bool PolyTrajOptimizer::getFormationPos(vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos)
  {
    if (swarm_trajs_->size() < formation_size_ || !use_formation_)
    {
      return false;
    }
    else
    {
      // Ensure swarm_graph_pos has the correct size
      if (static_cast<int>(swarm_graph_pos.size()) != formation_size_) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Resizing swarm_graph_pos from %zu to %d", 
                    swarm_graph_pos.size(), formation_size_);
        swarm_graph_pos.resize(formation_size_, Eigen::Vector3d::Zero());
      }
      
      // Bounds check for drone_id_
      if (drone_id_ < 0 || drone_id_ >= formation_size_) {
        RCLCPP_ERROR(node_->get_logger(), "drone_id_ %d out of bounds (formation_size: %d)", 
                     drone_id_, formation_size_);
        return false;
      }
      
      double pt_time = t_now_;
      swarm_graph_pos[drone_id_] = pos;
      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if (swarm_trajs_->at(id).drone_id < 0 || swarm_trajs_->at(id).drone_id == drone_id_)
          continue;
        double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
        Eigen::Vector3d swarm_p, swarm_v;
        if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
          swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                    exceed_time * swarm_v;
        }
        swarm_graph_pos[id] = swarm_p;
      }
      return true;
    }
  }

  void PolyTrajOptimizer::showFormationInformation(bool is_show, Eigen::Vector3d pos)
  {
    if (!is_show)
      return;

    if (swarm_trajs_->size() < formation_size_ || drone_id_ != 0 || !use_formation_)
      return;
    else
    {
      vector<Eigen::Vector3d> swarm_graph_pos(formation_size_);
      if (getFormationPos(swarm_graph_pos, pos)) {
        if (!swarm_graph_->updateGraph(swarm_graph_pos)) {
          RCLCPP_DEBUG(node_->get_logger(), "Failed to update swarm graph in showFormationInformation");
        }
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "Failed to get formation positions in showFormationInformation");
      }
    }
  }

  void PolyTrajOptimizer::setParam(const rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    node_->declare_parameter("optimization/constrain_points_perPiece", 3);
    node_->get_parameter("optimization/constrain_points_perPiece", cps_num_prePiece_);
    
    node_->declare_parameter("enable_obstacles", true);
    node_->get_parameter("enable_obstacles", enable_obstacles_);
    
    // Get enable_debug_logs parameter (declared in replan_fsm)
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);
    
    // Get enable_lbfgs_detail_logs parameter
    node_->get_parameter("enable_lbfgs_detail_logs", enable_lbfgs_detail_logs_);
    
    // Use conditional logging - only RCLCPP when debug logs disabled, only LogManager when enabled
    if (!enable_debug_logs_) {
        RCLCPP_INFO(node_->get_logger(), "Obstacle avoidance: %s", enable_obstacles_ ? "enabled" : "disabled");
        RCLCPP_INFO(node_->get_logger(), "Debug logging: disabled (using RCLCPP only)");
    }
    node_->declare_parameter("optimization/weight_obstacle", 1000.0);
    node_->get_parameter("optimization/weight_obstacle", wei_obs_);
    node_->declare_parameter("optimization/weight_swarm", 0.0);
    node_->get_parameter("optimization/weight_swarm", wei_swarm_);
    node_->declare_parameter("optimization/weight_feasibility", 1.0);
    node_->get_parameter("optimization/weight_feasibility", wei_feas_);
    node_->declare_parameter("optimization/weight_sqrvariance", 1.0);
    node_->get_parameter("optimization/weight_sqrvariance", wei_sqrvar_);
    node_->declare_parameter("optimization/weight_time", 0.0);
    node_->get_parameter("optimization/weight_time", wei_time_);
    node_->declare_parameter("optimization/weight_formation", 0.0);
    node_->get_parameter("optimization/weight_formation", wei_formation_);
    wei_formation_base_ = wei_formation_;  // Store base weight for adaptive adjustment

    // Only declare if not already declared (may be declared by ReplanFSM for dynamic changes)
    if (!node_->has_parameter("optimization/weight_nonholonomic")) {
        node_->declare_parameter("optimization/weight_nonholonomic", 15000.0);
    }
    node_->get_parameter("optimization/weight_nonholonomic", wei_nonholo_);

    node_->declare_parameter("optimization/obstacle_clearance", 0.1);
    node_->get_parameter("optimization/obstacle_clearance", obs_clearance_);
    node_->declare_parameter("optimization/swarm_clearance", 0.5);
    node_->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node_->declare_parameter("optimization/max_vel", 1.0);
    node_->get_parameter("optimization/max_vel", max_vel_);
    node_->declare_parameter("optimization/max_acc", 1.0);
    node_->get_parameter("optimization/max_acc", max_acc_);

    // Nonholonomic constraint parameters
    node_->declare_parameter("optimization/min_forward_vel", 0.2);
    node_->get_parameter("optimization/min_forward_vel", min_forward_vel_);
    node_->declare_parameter("optimization/max_brake_decel", 2.0);
    node_->get_parameter("optimization/max_brake_decel", max_brake_decel_);
    node_->declare_parameter("optimization/max_curvature", 0.8);
    node_->get_parameter("optimization/max_curvature", max_curvature_);
    node_->declare_parameter("optimization/max_lateral_accel", 1.5);
    node_->get_parameter("optimization/max_lateral_accel", max_lateral_accel_);

    // Log initialization based on enable_debug_logs setting
    if (enable_debug_logs_) {
        if (log_manager_) {
            log_manager_->infof("PolyTrajOptimizer parameters initialized");
            log_manager_->infof("Obstacle avoidance: %s", enable_obstacles_ ? "enabled" : "disabled");
            log_manager_->infof("Debug logging: enabled (using LogManager)");
            log_manager_->infof("L-BFGS detail logging: %s", enable_lbfgs_detail_logs_ ? "enabled" : "disabled");
        }
    }

    swarm_graph_.reset(new SwarmGraph);
    
    // Set initial formation based on optimizer_params.yaml
    int initial_formation_type = 2; // Default to REGULAR_SQUARE
    node_->declare_parameter("optimization/formation_type", initial_formation_type);
    node_->get_parameter("optimization/formation_type", initial_formation_type);
    
    LOG_INFO("Setting initial formation type: %d", initial_formation_type);
    setDesiredFormation(initial_formation_type);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;
    a_star_.reset(new AStar);
    
    // Set log manager for A* if available
    if (log_manager_) {
      a_star_->setLogManager(log_manager_);
    }
    
    // Calculate pool size based on map size and resolution
    Eigen::Vector3d map_size = grid_map_->getMapSize();
    double resolution = grid_map_->getResolution();
    
    Eigen::Vector3i pool_size(800, 800, 20);

    a_star_->initGridMap(grid_map_, pool_size);
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr)
  {
    swarm_trajs_ = swarm_trajs_ptr;
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id)
  {
    drone_id_ = drone_id;
  }

  void PolyTrajOptimizer::setFormation(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size)
  {
    // Safety check: ensure node_ is properly initialized before using logger
    if (!node_) {
      LOG_ERROR("PolyTrajOptimizer node_ is null in setFormation!");
      return;
    }
    
    if (swarm_graph_ && !formation_positions.empty()) {
      formation_size_ = formation_size;
      
      // Ensure formation positions match expected formation size
      std::vector<Eigen::Vector3d> adjusted_formation = formation_positions;
      if (static_cast<int>(adjusted_formation.size()) != formation_size_) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Formation positions size (%zu) doesn't match formation_size (%d), adjusting...", 
                    adjusted_formation.size(), formation_size_);
        adjusted_formation.resize(formation_size_, Eigen::Vector3d::Zero());
      }
      
      swarm_graph_->setDesiredForm(adjusted_formation);
      use_formation_ = true;
      
      LOG_INFO("Formation set with %d positions for optimizer (drone %d)", 
               static_cast<int>(adjusted_formation.size()), drone_id_);
      
      // Print current formation details
      LOG_INFO("=== CURRENT FORMATION CONFIGURATION ===");
      LOG_INFO("Formation Size: %d", formation_size_);
      LOG_INFO("Drone ID: %d", drone_id_);
      LOG_INFO("Formation Positions:");
      for (size_t i = 0; i < adjusted_formation.size(); ++i) {
        LOG_INFO("  Drone %zu: [%.3f, %.3f, %.3f]", 
                 i, adjusted_formation[i].x(), adjusted_formation[i].y(), adjusted_formation[i].z());
      }
      LOG_INFO("=====================================");
    } else {
      use_formation_ = false;
      formation_size_ = 0;
      LOG_WARN("Failed to set formation - swarm_graph not initialized or empty positions");
    }
  }

  void PolyTrajOptimizer::setDesiredFormation(int type)
  {
    std::vector<Eigen::Vector3d> swarm_des;
    switch (type)
    {
      case FORMATION_TYPE::NONE_FORMATION:
      {
        use_formation_ = false;
        formation_size_ = 0;
        break;
      }

      case FORMATION_TYPE::REGULAR_HEXAGON:
      {
        // set the desired formation
        Eigen::Vector3d v0(0, 0, 0);
        Eigen::Vector3d v1(1.7321, -1, 0);
        Eigen::Vector3d v2(0, -2, 0);
        Eigen::Vector3d v3(-1.7321, -1, 0);
        Eigen::Vector3d v4(-1.7321, 1, 0);
        Eigen::Vector3d v5(0, 2, 0);
        Eigen::Vector3d v6(1.7321, 1, 0);

        swarm_des.push_back(v0);
        swarm_des.push_back(v1);
        swarm_des.push_back(v2);
        swarm_des.push_back(v3);
        swarm_des.push_back(v4);
        swarm_des.push_back(v5);
        swarm_des.push_back(v6);

        formation_size_ = swarm_des.size();
        // construct the desired swarm graph
        if (swarm_graph_) {
          swarm_graph_->setDesiredForm(swarm_des);
        }
        break;
      }

      case FORMATION_TYPE::REGULAR_SQUARE:
      {
        // Square formation
        Eigen::Vector3d v0(0, 0, 0);
        Eigen::Vector3d v1(1, 0, 0);
        Eigen::Vector3d v2(1, 1, 0);
        Eigen::Vector3d v3(0, 1, 0);

        swarm_des.push_back(v0);
        swarm_des.push_back(v1);
        swarm_des.push_back(v2);
        swarm_des.push_back(v3);

        formation_size_ = swarm_des.size();
        if (swarm_graph_) {
          swarm_graph_->setDesiredForm(swarm_des);
        }
        break;
      }

      default:
        use_formation_ = false;
        formation_size_ = 0;
        break;
    }
  }

  // Compute total jerk: ∫ ||d³P/dt³||² dt
  double PolyTrajOptimizer::computeTotalJerk(const poly_traj::Trajectory &traj)
  {
    double total_jerk = 0.0;
    double dt = 0.01;  // Sample every 10ms
    double T = traj.getTotalDuration();
    int num_samples = static_cast<int>(T / dt);

    for (int i = 0; i < num_samples; ++i)
    {
      double t = i * dt;
      Eigen::Vector3d jerk = traj.getJer(t);  // Get jerk at time t
      total_jerk += jerk.squaredNorm() * dt;  // Integrate ||jerk||² * dt
    }

    return total_jerk;
  }

  // Compute maximum jerk: max_{t∈[0,T]} ||d³P/dt³(t)||
  double PolyTrajOptimizer::computeMaxJerk(const poly_traj::Trajectory &traj)
  {
    double max_jerk = 0.0;
    double dt = 0.01;  // Sample every 10ms
    double T = traj.getTotalDuration();
    int num_samples = static_cast<int>(T / dt);

    for (int i = 0; i < num_samples; ++i)
    {
      double t = i * dt;
      Eigen::Vector3d jerk = traj.getJer(t);
      double jerk_norm = jerk.norm();

      if (jerk_norm > max_jerk)
      {
        max_jerk = jerk_norm;
      }
    }

    return max_jerk;
  }
}
