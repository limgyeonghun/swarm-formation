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

    // Build piece-to-polytope mapping
    if (!sfc_hpolys_.empty()) {
      buildPiecePolytopeMapping(piece_num_);
    }

    // Build V-polytope mapping if V-polytope parameterization is active
    bool vpoly_active = use_vpoly_param_ && !sfc_vpolys_.empty() && !sfc_hpolys_.empty();
    if (vpoly_active) {
      buildVPolyMapping(piece_num_);
      if (spatial_dim_ <= 0 || vpoly_idx_.size() != piece_num_ - 1) {
        if (log_manager_) {
          log_manager_->warnf("[V-POLY] buildVPolyMapping failed (spatial_dim=%d, vpoly_idx=%d), falling back to legacy",
                              spatial_dim_, (int)vpoly_idx_.size());
        }
        vpoly_active = false;
      }
    }

    Eigen::Vector3d start_pos = iniState.col(0);

    double final_cost;

    auto t0 = node_->get_clock()->now();
    auto t1 = node_->get_clock()->now();
    auto t2 = node_->get_clock()->now();

    // L-BFGS parameter setup (GCOPTER-style: converge until cost stops improving)
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 256;
    lbfgs_params.g_epsilon = 0.0;          // Disable gradient norm test
    lbfgs_params.past = 3;                 // Compare cost with 3 iterations ago
    lbfgs_params.delta = 1.0e-5;           // Stop when relative cost change < 1e-5
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.max_iterations = 0;       // Unlimited (GCOPTER-style: converge by delta only)

    if (!use_formation)
    {
      use_formation_ = false;
    }

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    int result;

    if (vpoly_active)
    {
      // === V-POLYTOPE PATH: xi parameterization ===
      // Variable layout: x = [tau(piece_num_) | xi(spatial_dim_)]
      variable_num_ = piece_num_ + spatial_dim_;
      std::vector<double> q(variable_num_);

      // Initialize tau (virtual time) from initT
      Eigen::Map<Eigen::VectorXd> tau_init(q.data(), piece_num_);
      RealT2VirtualT(initT, tau_init);

      // Initialize xi by projecting initial 3D inner points to V-polytope space
      Eigen::Matrix3Xd initP = initInnerPts;
      backwardP(initP, vpoly_idx_, sfc_vpolys_, q.data() + piece_num_);

      if (log_manager_ && enable_debug_logs_) {
          log_manager_->infof("[V-POLY] Optimization: variable_num=%d (tau=%d + xi=%d), pieces=%d",
                              variable_num_, piece_num_, spatial_dim_, piece_num_);
      }

      t1 = node_->get_clock()->now();

      result = lbfgs::lbfgs_optimize(
          variable_num_,
          q.data(),
          &final_cost,
          PolyTrajOptimizer::costFunctionCallbackVPoly,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      // Extract final trajectory from optimized variables
      Eigen::Map<const Eigen::VectorXd> tau_final(q.data(), piece_num_);
      Eigen::VectorXd T_final(piece_num_);
      VirtualT2RealT(tau_final, T_final);
      Eigen::Matrix3Xd P_final;
      forwardP(q.data() + piece_num_, vpoly_idx_, sfc_vpolys_, P_final);
      jerkOpt_.generate(P_final, T_final);
    }
    else
    {
      // === LEGACY PATH: direct 3D coordinate optimization ===
      variable_num_ = 4 * (piece_num_ - 1) + 1;
      std::vector<double> q(variable_num_);
      memcpy(q.data(), initInnerPts.data(), initInnerPts.size() * sizeof(double));
      Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
      RealT2VirtualT(initT, Vt);

      t1 = node_->get_clock()->now();

      result = lbfgs::lbfgs_optimize(
          variable_num_,
          q.data(),
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);
    }

    // Log L-BFGS result
    if (log_manager_ && enable_debug_logs_) {
        const char* result_str = lbfgs::lbfgs_strerror(result);
        log_manager_->infof("L-BFGS Result: %d (%s), mode=%s", result, result_str,
                            vpoly_active ? "V-POLY" : "LEGACY");
        log_manager_->infof("Iteration info: costFunction calls=%d, max_iterations=%d", iter_num_, lbfgs_params.max_iterations);
    }

    // Collision check (only if obstacles are enabled)
    bool occ = enable_obstacles_ ? checkCollision() : false;

    t2 = node_->get_clock()->now();
    double time_ms = (t2 - t1).seconds() * 1000;
    double total_time_ms = (t2 - t0).seconds() * 1000;

    // Final result logging
    if (log_manager_) {
        log_manager_->infof("Optimization completed: iter=%d, use_formation_param=%d, use_formation_internal=%d, time(ms)=%f",
                            iter_num_, use_formation, use_formation_, time_ms);

        // Calculate and log jerk metrics
        poly_traj::Trajectory final_traj = jerkOpt_.getTraj();
        double total_jerk = computeTotalJerk(final_traj);
        double max_jerk = computeMaxJerk(final_traj);
        log_manager_->infof("[JERK METRICS] total_jerk=%.6f, max_jerk=%.6f m/s³, duration=%.3f s",
                            total_jerk, max_jerk, final_traj.getTotalDuration());

        log_manager_->infof("[COST] formation_cost=%f (wei_formation=%f, similarity=%f)", dbg_cost_formation_, wei_formation_, debug_similarity_);

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

    if (!sfc_hpolys_.empty())
    {
      // SFC corridor-based collision check
      for (int i = 0; i < i_end; i++)
      {
        Eigen::Vector3d pos = traj.getPos(t);

        // Check if point is inside any polytope in the corridor
        bool inside_corridor = false;
        for (const auto &hp : sfc_hpolys_)
        {
          Eigen::VectorXd viola = hp.leftCols<3>() * pos + hp.rightCols<1>();
          // 부동소수점 노이즈로 start/goal이 경계 위에서 살짝 밖으로 판정되는 것 방지
          if (viola.maxCoeff() <= 1.0e-4)
          {
            inside_corridor = true;
            break;
          }
        }

        if (!inside_corridor)
        {
          LOG_WARN("[COLLISION] t=%.3f pos=(%.3f, %.3f, %.3f) outside SFC corridor",
                   t, pos.x(), pos.y(), pos.z());
          occ = true;
          break;
        }

        t += dt;
      }
    }
    else
    {
      LOG_WARN("[COLLISION] No SFC corridor available for collision check");
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
        opt->log_manager_->infof("  threat_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(3), opt->wei_threat_);
        opt->log_manager_->infof("  feasibility_cost=%.6f (weight=%.3f)", obs_swarm_feas_qvar_costs(4), opt->wei_feas_);
        opt->log_manager_->infof("  time_cost=%.6f (weight=%.3f)", time_cost, opt->wei_time_);

        // Store formation cost for final logging
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    } else if (!opt->enable_lbfgs_detail_logs_ && opt->use_formation_) {
        // Store formation cost for final logging even when detail logs are disabled
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    }

    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  // V-Polytope parameterization cost function callback.
  // Variable layout: x = [tau(N) | xi(spatial_dim)]
  // Key difference from legacy: xi -> P via forwardP (guaranteed inside SFC),
  // gradients flow back via backwardGradP. No corridor penalty needed.
  double PolyTrajOptimizer::costFunctionCallbackVPoly(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    const int dimTau = opt->piece_num_;
    const int dimXi = opt->spatial_dim_;

    // Map optimization variables: [tau | xi]
    const double *tau_data = x;
    const double *xi_data = x + dimTau;
    double *gradTau_data = grad;
    double *gradXi_data = grad + dimTau;

    // 1. Forward transform: tau -> T (real durations)
    Eigen::Map<const Eigen::VectorXd> tau(tau_data, dimTau);
    Eigen::VectorXd T(opt->piece_num_);
    opt->VirtualT2RealT(tau, T);

    // 2. Forward transform: xi -> P (3D inner points, guaranteed inside SFC)
    Eigen::Matrix3Xd P;
    forwardP(xi_data, opt->vpoly_idx_, opt->sfc_vpolys_, P);

    // 3. Generate MINCO trajectory
    opt->jerkOpt_.generate(P, T);

    // 4. Smoothness (jerk energy) cost + gradient
    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0;
    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost);

    // 5. Penalty costs: feasibility + swarm + formation (NO corridor, NO sqrvariance)
    Eigen::VectorXd obs_swarm_feas_qvar_costs(6);
    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_);

    // 6. Adjoint gradient propagation: gdC -> gradP (gradient w.r.t. inner points)
    Eigen::Matrix3Xd gradP(3, opt->piece_num_ - 1);
    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    // 7. Backward transform: gradP -> gradXi (chain rule through V-polytope parameterization)
    memset(gradXi_data, 0, dimXi * sizeof(double));
    backwardGradP(xi_data, opt->vpoly_idx_, opt->sfc_vpolys_, gradP, gradXi_data);

    // 8. Norm restriction layer: soft penalty to keep ||xi|| near 1
    double total_cost = smoo_cost + obs_swarm_feas_qvar_costs.sum();
    normRestrictionLayer(xi_data, opt->vpoly_idx_, opt->sfc_vpolys_, total_cost, gradXi_data);

    // 9. Time cost + gradient (virtual time parameterization)
    double time_cost = 0;
    Eigen::Map<Eigen::VectorXd> gradTauMap(gradTau_data, dimTau);
    opt->VirtualTGradCost(T, tau, gradT, gradTauMap, time_cost);

    opt->iter_num_ += 1;

    // Debug logging
    if (opt->enable_lbfgs_detail_logs_ && opt->log_manager_ && opt->iter_num_ % 10 == 0) {
        double final_cost = total_cost + time_cost;
        opt->log_manager_->infof("[V-POLY L-BFGS] iter=%d, total=%.6f, smooth=%.6f, feas=%.6f, swarm=%.6f, form=%.6f, threat=%.6f, time=%.6f",
            opt->iter_num_, final_cost, smoo_cost,
            obs_swarm_feas_qvar_costs(4), obs_swarm_feas_qvar_costs(1),
            obs_swarm_feas_qvar_costs(2), obs_swarm_feas_qvar_costs(3), time_cost);
    }

    if (opt->use_formation_) {
        opt->dbg_cost_formation_ = obs_swarm_feas_qvar_costs(2);
    }

    return total_cost + time_cost;
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

        // Obstacle/Corridor cost calculation (SFC corridor-based)
        // Skip when V-polytope parameterization is active: corridor is structurally guaranteed
        if (enable_obstacles_ && !sfc_hpolys_.empty() && !use_vpoly_param_) {
            bool has_corridor_cost = corridorGradCostP(i, pos, gradp, costp);
            if (has_corridor_cost) {
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

        // Threat zone cost calculation (soft constraint for air defense penetration)
        if (use_threat_zones_ && threatGradCostP(i_dp, pos, gradp, costp)) {
            gradViolaPc = beta0 * gradp.transpose();
            gradViolaPt = alpha * gradp.transpose() * vel;
            jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
            gdT(i) += omg * (costp / K + step * gradViolaPt);
            costs(3) += omg * step * costp;
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

        s1 += step;
        if (j != K || (j == K && i == N - 1)) {
            ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // Distance variance cost: skip when V-polytope parameterization is active
    // (not in original GCOPTER; MINCO jerk minimization provides sufficient smoothness)
    if (!use_vpoly_param_) {
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
    }

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
    if (i_dp <= 0 || i_dp >= cps_.cp_size)
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

  void PolyTrajOptimizer::buildPiecePolytopeMapping(int piece_num)
  {
    // GCOPTER-style: distribute pieces evenly across polytopes
    int polyN = sfc_hpolys_.size();
    if (polyN == 0 || piece_num <= 0)
    {
      hpoly_piece_idx_.resize(0);
      return;
    }

    hpoly_piece_idx_.resize(piece_num);

    if (polyN == 1)
    {
      // All pieces map to the single polytope
      hpoly_piece_idx_.setZero();
      return;
    }

    // path_manager가 계산한 pieces_per_poly_가 있으면 그대로 사용 (GCOPTER 원본 방식).
    // 없으면 fallback으로 round-robin 분배.
    Eigen::VectorXi piecesPerPoly;
    if (pieces_per_poly_.size() == polyN && pieces_per_poly_.sum() == piece_num) {
      piecesPerPoly = pieces_per_poly_;
    } else {
      piecesPerPoly = Eigen::VectorXi::Ones(polyN);
      int remaining = piece_num - polyN;
      for (int i = 0; i < remaining; ++i) {
        piecesPerPoly(i % polyN) += 1;
      }
    }

    int j = 0;
    for (int i = 0; i < polyN; ++i)
    {
      for (int k = 0; k < piecesPerPoly(i) && j < piece_num; ++k, ++j)
      {
        hpoly_piece_idx_(j) = i;
      }
    }
  }

  bool PolyTrajOptimizer::corridorGradCostP(const int piece_idx,
                                             const Eigen::Vector3d &p,
                                             Eigen::Vector3d &gradp,
                                             double &costp)
  {
    gradp.setZero();
    costp = 0;

    if (sfc_hpolys_.empty()) return false;

    // GCOPTER-style: use piece-to-polytope mapping
    int poly_idx;
    if (hpoly_piece_idx_.size() > 0 && piece_idx >= 0 && piece_idx < hpoly_piece_idx_.size())
    {
      poly_idx = hpoly_piece_idx_(piece_idx);
    }
    else
    {
      // Fallback: find closest polytope (legacy behavior)
      double best_max_violation = std::numeric_limits<double>::max();
      poly_idx = 0;
      for (size_t k = 0; k < sfc_hpolys_.size(); ++k)
      {
        Eigen::VectorXd violations = sfc_hpolys_[k].leftCols<3>() * p + sfc_hpolys_[k].rightCols<1>();
        double max_viola = violations.maxCoeff();
        if (max_viola < best_max_violation)
        {
          best_max_violation = max_viola;
          poly_idx = k;
        }
      }
    }

    // GCOPTER-style: smoothedL1 penalty for each violating face of assigned polytope
    bool ret = false;
    const Eigen::MatrixX4d &hPoly = sfc_hpolys_[poly_idx];
    int K = hPoly.rows();

    for (int k = 0; k < K; ++k)
    {
      Eigen::Vector3d outerNormal = hPoly.row(k).head<3>();
      double violaPos = outerNormal.dot(p) + hPoly(k, 3);

      double violaPosPena, violaPosPenaD;
      if (smoothedL1(violaPos, smoothing_eps_, violaPosPena, violaPosPenaD))
      {
        gradp += wei_obs_ * violaPosPenaD * outerNormal;
        costp += wei_obs_ * violaPosPena;
        ret = true;
      }
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
    if (i_dp <= 0 || i_dp >= cps_.cp_size)
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
    // GCOPTER-style: smoothedL1 penalty for each velocity component
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    double violaVelPena, violaVelPenaD;
    if (smoothedL1(vpen, smoothing_eps_, violaVelPena, violaVelPenaD))
    {
      gradv = wei_feas_ * violaVelPenaD * 2.0 * v;
      costv = wei_feas_ * violaVelPena;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    // GCOPTER-style: smoothedL1 penalty for acceleration
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    double violaAccPena, violaAccPenaD;
    if (smoothedL1(apen, smoothing_eps_, violaAccPena, violaAccPenaD))
    {
      grada = wei_feas_ * violaAccPenaD * 2.0 * a;
      costa = wei_feas_ * violaAccPena;
      return true;
    }
    return false;
  }

  // Continuous Gaussian threat over detection range (no engagement/detection split).
  // Matches ObstacleQueryAdapter::getThreatLevel in path_manager.h.
  double PolyTrajOptimizer::getThreatLevel(const Eigen::Vector3d &pos) const
  {
    double total_threat = 0.0;
    for (const auto &tz : threat_zones_) {
      double dist = (pos - tz.center).norm();
      if (dist < tz.detection_range) {
        double sigma = tz.detection_range / 3.0;
        total_threat += tz.max_threat_level * std::exp(-0.5 * (dist / sigma) * (dist / sigma));
      }
    }
    return total_threat;
  }

  Eigen::Vector3d PolyTrajOptimizer::getThreatGradient(const Eigen::Vector3d &pos) const
  {
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    for (const auto &tz : threat_zones_) {
      Eigen::Vector3d diff = pos - tz.center;
      double dist = diff.norm();
      if (dist < 1e-6) continue;
      if (dist < tz.detection_range) {
        double sigma = tz.detection_range / 3.0;
        double sigma2 = sigma * sigma;
        double gauss = tz.max_threat_level * std::exp(-0.5 * (dist / sigma) * (dist / sigma));
        // ∇threat = gauss × (−dist/σ²) × (diff/dist) = gauss × (−1/σ²) × diff
        grad += gauss * (-1.0 / sigma2) * diff;
      }
    }
    return grad;
  }

  bool PolyTrajOptimizer::threatGradCostP(const int i_dp,
                                           const Eigen::Vector3d &p,
                                           Eigen::Vector3d &gradp,
                                           double &costp)
  {
    if (i_dp <= 0 || i_dp >= cps_.cp_size)
      return false;

    double threat = getThreatLevel(p);
    if (threat > 0.01) {
      Eigen::Vector3d threat_grad = getThreatGradient(p);
      // Quadratic cost: smooth penalty proportional to threat^2
      costp = wei_threat_ * threat * threat;
      gradp = wei_threat_ * 2.0 * threat * threat_grad;
      return true;
    }
    return false;
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

    node_->declare_parameter("optimization/weight_threat", 0.0);
    node_->get_parameter("optimization/weight_threat", wei_threat_);

    node_->declare_parameter("optimization/swarm_clearance", 0.5);
    node_->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node_->declare_parameter("optimization/max_vel", 1.0);
    node_->get_parameter("optimization/max_vel", max_vel_);
    node_->declare_parameter("optimization/max_acc", 1.0);
    node_->get_parameter("optimization/max_acc", max_acc_);

    // GCOPTER-style smoothedL1 smoothing factor
    node_->declare_parameter("optimization/smoothing_eps", 0.01);
    node_->get_parameter("optimization/smoothing_eps", smoothing_eps_);

    // V-polytope parameterization (GCOPTER-style: structural corridor guarantee)
    node_->declare_parameter("optimization/use_vpoly_param", true);
    node_->get_parameter("optimization/use_vpoly_param", use_vpoly_param_);

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

      // Check if this is NONE mode (all positions are zero)
      bool is_none_mode = true;
      for (const auto& pos : adjusted_formation) {
        if (pos.norm() > 1e-6) {  // Non-zero position found
          is_none_mode = false;
          break;
        }
      }

      if (is_none_mode) {
        // NONE mode: disable formation cost
        use_formation_ = false;
        wei_formation_ = 0.0;
        LOG_INFO("NONE mode detected - formation cost DISABLED (weight=0)");
      } else {
        // Normal formation mode
        swarm_graph_->setDesiredForm(adjusted_formation);
        use_formation_ = true;
        wei_formation_ = wei_formation_base_;  // Restore base weight

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
      }
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

  /* ================================================================
   *  GCOPTER V-Polytope Parameterization Functions
   *  Ported from: gcopter.hpp (ZJU-FAST-Lab/GCOPTER)
   *  Purpose: Guarantee trajectory control points stay inside SFC
   *           corridor via structural parameterization, not penalty.
   * ================================================================ */

  // Convert xi weights to 3D positions via V-polytope vertices.
  // Each inner point is a convex combination of polytope vertices:
  //   P = V.col(0) + V.rightCols(k-1) * q²  where q = xi.normalized()
  // Since q² >= 0, the point is guaranteed inside the polytope.
  // Ported from gcopter.hpp:143-160
  void PolyTrajOptimizer::forwardP(const double *xi_data,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
  {
    const int sizeP = vIdx.size();
    P.resize(3, sizeP);
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
    {
      l = vIdx(i);
      k = vPolys[l].cols();
      Eigen::Map<const Eigen::VectorXd> seg(xi_data + j, k);
      Eigen::VectorXd q = seg.normalized().head(k - 1);
      P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                  vPolys[l].col(0);
    }
  }

  // Chain-rule gradient: gradP (w.r.t. 3D positions) -> gradXi (w.r.t. xi weights).
  // Ported from gcopter.hpp:236-262
  void PolyTrajOptimizer::backwardGradP(const double *xi_data,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         double *gradXi_data)
  {
    const int sizeP = vIdx.size();
    double normInv;
    Eigen::VectorXd q, gradQ, unitQ;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
    {
      l = vIdx(i);
      k = vPolys[l].cols();
      Eigen::Map<const Eigen::VectorXd> seg(xi_data + j, k);
      Eigen::Map<Eigen::VectorXd> gradSeg(gradXi_data + j, k);
      normInv = 1.0 / seg.norm();
      unitQ = seg * normInv;
      gradQ.resize(k);
      gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                            unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradSeg = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
    }
  }

  // Soft penalty to keep ||xi_segment|| near 1.
  // If ||q||² > 1, adds penalty = (||q||²-1)³ to prevent runaway norm.
  // Ported from gcopter.hpp:265-294
  void PolyTrajOptimizer::normRestrictionLayer(const double *xi_data,
                                                const Eigen::VectorXi &vIdx,
                                                const PolyhedraV &vPolys,
                                                double &cost,
                                                double *gradXi_data)
  {
    const int sizeP = vIdx.size();
    for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
    {
      k = vPolys[vIdx(i)].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi_data + j, k);
      Eigen::Map<Eigen::VectorXd> gradQ(gradXi_data + j, k);
      double sqrNormQ = q.squaredNorm();
      double sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0)
      {
        double c = sqrNormViolation * sqrNormViolation;
        double dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradQ += dc * 2.0 * q;
      }
    }
  }

  // Per-point NLS cost for backwardP: find xi that minimizes ||forwardP(xi) - target||².
  // Ported from gcopter.hpp:162-193
  double PolyTrajOptimizer::costTinyNLS(void *ptr,
                                         const double *x, double *grad, const int n)
  {
    const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;
    Eigen::Map<const Eigen::VectorXd> xi(x, n);
    Eigen::Map<Eigen::VectorXd> gradXi(grad, n);

    const double sqrNormXi = xi.squaredNorm();
    const double invNormXi = 1.0 / sqrt(sqrNormXi);
    const Eigen::VectorXd unitXi = xi * invNormXi;
    const Eigen::VectorXd r = unitXi.head(n - 1);
    const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                  ovPoly.col(1) - ovPoly.col(0);

    double cost = delta.squaredNorm();
    gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                           r.array() * 2.0;
    gradXi(n - 1) = 0.0;
    gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

    const double sqrNormViolation = sqrNormXi - 1.0;
    if (sqrNormViolation > 0.0)
    {
      double c = sqrNormViolation * sqrNormViolation;
      const double dc = 3.0 * c;
      c *= sqrNormViolation;
      cost += c;
      gradXi += dc * 2.0 * xi;
    }

    return cost;
  }

  // Project 3D inner points to xi space by solving per-point NLS.
  // Ported from gcopter.hpp:196-233
  void PolyTrajOptimizer::backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     double *xi_data)
  {
    const int sizeP = P.cols();

    lbfgs::lbfgs_parameter_t tiny_nls_params;
    lbfgs::lbfgs_load_default_parameters(&tiny_nls_params);
    tiny_nls_params.past = 0;
    tiny_nls_params.delta = 1.0e-5;
    tiny_nls_params.g_epsilon = 1.0e-6;
    tiny_nls_params.max_iterations = 128;

    Eigen::Matrix3Xd ovPoly;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
    {
      l = vIdx(i);
      k = vPolys[l].cols();

      // Build ovPoly: col(0)=target point, col(1)=reference vertex, cols(2+)=offset vertices
      ovPoly.resize(3, k + 1);
      ovPoly.col(0) = P.col(i);
      ovPoly.rightCols(k) = vPolys[l];

      // Initialize xi segment uniformly
      Eigen::Map<Eigen::VectorXd> seg(xi_data + j, k);
      seg.setConstant(sqrt(1.0 / k));

      double minSqrD;
      lbfgs::lbfgs_optimize(k, seg.data(), &minSqrD,
                             &PolyTrajOptimizer::costTinyNLS,
                             nullptr, nullptr,
                             &ovPoly, &tiny_nls_params);
    }
  }

  // Build mapping: each inner point -> which V-polytope it belongs to.
  // For N corridor polytopes: 2*N+1 V-polytopes (individual + overlaps).
  // Interior piece boundaries map to individual V-polytopes (even indices),
  // polytope-crossing boundaries map to overlap V-polytopes (odd indices).
  // Mirrors gcopter.hpp:772-791
  void PolyTrajOptimizer::buildVPolyMapping(int piece_num)
  {
    int polyN = sfc_hpolys_.size();
    if (polyN == 0 || piece_num <= 0 || sfc_vpolys_.empty()) {
      vpoly_idx_.resize(0);
      spatial_dim_ = 0;
      return;
    }

    // path_manager가 계산한 pieces_per_poly_가 있으면 그대로 사용.
    Eigen::VectorXi piecesPerPoly;
    if (pieces_per_poly_.size() == polyN && pieces_per_poly_.sum() == piece_num) {
      piecesPerPoly = pieces_per_poly_;
    } else {
      piecesPerPoly = Eigen::VectorXi::Ones(polyN);
      if (piece_num > polyN) {
        int remaining = piece_num - polyN;
        for (int i = 0; i < remaining; ++i) {
          piecesPerPoly(i % polyN) += 1;
        }
      }
    }

    // Build vPolyIdx for piece_num - 1 inner points
    // Mirrors GCOPTER gcopter.hpp:772-791 exactly
    vpoly_idx_.resize(piece_num - 1);
    spatial_dim_ = 0;
    for (int i = 0, j = 0, k; i < polyN; i++)
    {
      k = piecesPerPoly(i);
      for (int l = 0; l < k; l++, j++)
      {
        if (l < k - 1)
        {
          // Interior point: belongs to individual V-polytope (even index)
          vpoly_idx_(j) = 2 * i;
          spatial_dim_ += sfc_vpolys_[2 * i].cols();
        }
        else if (i < polyN - 1)
        {
          // Boundary point: belongs to overlap V-polytope (odd index)
          vpoly_idx_(j) = 2 * i + 1;
          spatial_dim_ += sfc_vpolys_[2 * i + 1].cols();
        }
        // Last piece of last polytope: no inner point (j does not index vPolyIdx)
      }
    }

    if (log_manager_ && enable_debug_logs_) {
      log_manager_->infof("[V-POLY] buildVPolyMapping: piece_num=%d, inner_points=%d, spatial_dim=%d, vpolys=%zu",
                          piece_num, piece_num - 1, spatial_dim_, sfc_vpolys_.size());
    }
  }

}
