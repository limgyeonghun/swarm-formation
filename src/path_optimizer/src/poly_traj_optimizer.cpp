#include "path_optimizer/poly_traj_optimizer.h"
#include <iomanip>
#include <ctime>
#include <sys/resource.h>

namespace ego_planner
{
  void PolyTrajOptimizer::initLogFile(const std::string &path)
  {
    log_file_path_ = path;
    log_file_.open(log_file_path_, std::ios::out | std::ios::app);
    if (!log_file_.is_open())
    {
      std::cerr << "Failed to open log file: " << log_file_path_ << std::endl;
    }
    else
    {
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      log_file_ << "=== Log started at " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << " ===\n";
    }
  }

  void PolyTrajOptimizer::closeLogFile()
  {
    if (log_file_.is_open())
    {
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      log_file_ << "=== Log ended at " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << " ===\n";
      log_file_.close();
    }
  }

  void PolyTrajOptimizer::logToFile(const std::string &message, const std::string &level)
  {
    if (log_file_.is_open())
    {
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      log_file_ << "[" << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "] [" << level << "] " << message << "\n";
      log_file_.flush();
    }
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

    // 메모리 사용량 체크
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    size_t current_memory = usage.ru_maxrss;
    
    // 메모리 사용량이 너무 높으면 최적화 중단
    const size_t MEMORY_LIMIT = 10000000; // 10GB (KB 단위)
    if (current_memory > MEMORY_LIMIT) {
        printf("[ERROR] Memory usage too high (%zu MB). Skipping trajectory optimization to prevent OOM.\n", 
               current_memory / 1024);
        return false;
    }

    t_now_ = node_->get_clock()->now().seconds();
    piece_num_ = initT.size();

    // 메모리 할당 크기 제한
    if (piece_num_ > 50) { // 최대 50개 세그먼트로 제한
        printf("[WARN] Too many trajectory pieces (%d). Limiting to 50 to prevent memory issues.\n", piece_num_);
        piece_num_ = 50;
    }

    jerkOpt_.reset(iniState, finState, piece_num_);
    Eigen::Vector3d start_pos = iniState.col(0);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    // 변수 수가 너무 많으면 제한
    if (variable_num_ > 1000) {
        printf("[ERROR] Too many optimization variables (%d). Aborting to prevent memory issues.\n", variable_num_);
        return false;
    }

    auto t0 = node_->get_clock()->now();
    auto t1 = node_->get_clock()->now();
    auto t2 = node_->get_clock()->now();
    bool use_formation_temp = use_formation_;

    std::vector<double> q;
    try {
        q.resize(variable_num_);
        memcpy(q.data(), initInnerPts.data(), initInnerPts.size() * sizeof(double));
        Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
        RealT2VirtualT(initT, Vt);
    } catch (const std::bad_alloc& e) {
        printf("[ERROR] Failed to allocate memory for optimization variables: %s\n", e.what());
        return false;
    }

    // 3. L-BFGS parameter setup
    auto t3 = node_->get_clock()->now();
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;

    if (use_formation)
    {
      lbfgs_params.max_iterations = 15;  // 메모리 절약을 위해 감소 (20->15)
      lbfgs_params.mem_size = 8;         // 메모리 사용량 감소 (16->8)
    }
    else
    {
      lbfgs_params.max_iterations = 40;  // 메모리 절약을 위해 감소 (60->40)
      lbfgs_params.mem_size = 12;        // 메모리 사용량 감소 (16->12)
      use_formation_ = false;
    }

    // Debug: Print L-BFGS parameters (similar to con code)
    if (enable_debug_logs_) {
        printf("[ INFO] [DEBUG] L-BFGS params: mem_size=%d, max_iter=%d, g_epsilon=%.2e, min_step=%.2e, use_formation=%d\n", 
               lbfgs_params.mem_size, lbfgs_params.max_iterations, lbfgs_params.g_epsilon, lbfgs_params.min_step, use_formation);
        fflush(stdout);  // Immediate output for debug info
    }

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    t1 = node_->get_clock()->now();

    // 4. L-BFGS optimization (main computation) - 메모리 안전 최적화
    auto t4 = node_->get_clock()->now();
    int result = -1;
    try {
        // 최적화 전 메모리 사용량 재확인
        getrusage(RUSAGE_SELF, &usage);
        size_t pre_opt_memory = usage.ru_maxrss;
        
        if (pre_opt_memory > MEMORY_LIMIT * 0.8) { // 80% 임계치
            printf("[WARN] Memory usage high before optimization (%zu MB). Using conservative settings.\n", 
                   pre_opt_memory / 1024);
            lbfgs_params.max_iterations = std::min(lbfgs_params.max_iterations, 10);
            lbfgs_params.mem_size = std::min(lbfgs_params.mem_size, 6);
        }
        
        result = lbfgs::lbfgs_optimize(
            variable_num_,
            q.data(),
            &final_cost,
            PolyTrajOptimizer::costFunctionCallback,
            nullptr,  // proc_stepbound (not used)
            PolyTrajOptimizer::earlyExitCallback,  // proc_progress
            this,
            &lbfgs_params);
            
        // 최적화 후 메모리 사용량 확인
        getrusage(RUSAGE_SELF, &usage);
        size_t post_opt_memory = usage.ru_maxrss;
        if (post_opt_memory > pre_opt_memory + 500000) { // 500MB 이상 증가
            printf("[WARN] Optimization increased memory usage significantly: %+zd MB\n", 
                   (post_opt_memory - pre_opt_memory) / 1024);
        }
    } catch (const std::exception& e) {
        printf("[ERROR] Exception during L-BFGS optimization: %s\n", e.what());
        return false;
    }
    // Log L-BFGS result (only for debugging)
    if (enable_debug_logs_) {
        const char* result_str = lbfgs::lbfgs_strerror(result);
        printf("[ INFO] [DEBUG] L-BFGS Result: %d (%s)\n", result, result_str);
        printf("[ INFO] [DEBUG] Iteration info: costFunction calls=%d, max_iterations=%d\n", 
               iter_num_, lbfgs_params.max_iterations);
        fflush(stdout);  // Immediate output for debug info
    }

    // Collision check (only if obstacles are enabled)
    bool occ = enable_obstacles_ ? checkCollision() : false;

    use_formation_ = use_formation_temp;

    t2 = node_->get_clock()->now();
    double time_ms = (t2 - t1).seconds() * 1000;
    double total_time_ms = (t2 - t0).seconds() * 1000;

    // Final result logging similar to con code
    printf("\033[32mid = %d, iter=%d, use_formation=%d, time(ms)=%5.3f\033[0m\n", drone_id_, iter_num_, use_formation, time_ms);
    fflush(stdout);  // Immediate output to sync with RCLCPP logs

    printf("[COST] formation_cost = %.6f (wei_formation=%.3f, similarity=%.6f)\n",
           dbg_cost_formation_, wei_formation_, debug_similarity_);
    fflush(stdout);

    // Additional debugging info (similar to con code)
    if (enable_debug_logs_) {
        printf("[ INFO] [DEBUG] L-BFGS Result: %d (%s)\n", result, lbfgs::lbfgs_strerror(result));
        printf("[ INFO] [DEBUG] Iteration info: costFunction calls=%d, max_iterations=%d, Final cost: %.6f\n", 
               iter_num_, lbfgs_params.max_iterations, final_cost);
        fflush(stdout);  // Immediate output for debug info
    }
    printf("=================================================\n");
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
    T_end = durations.head(piece_of_idx).sum() + durations(piece_of_idx) * (idx - piece_of_idx * cps_num_prePiece_) / cps_num_prePiece_;

    bool occ = false;
    double dt = 0.01;
    int i_end = floor(T_end / dt);
    double t = 0.0;
    collision_check_time_end_ = T_end;

    for (int i = 0; i < i_end; i++)
    {
      Eigen::Vector3d pos = traj.getPos(t);
      if (grid_map_->getInflateOccupancy(pos) == 1)
      {
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

    // Debug output similar to con code (using printf for speed)
    if (opt->iter_num_ % 50 == 0 && opt->enable_debug_logs_) {
        double total_callback_time = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        printf("[ INFO] [DEBUG] CostFunction iter=%d: Traj=%.2fms, Smooth=%.2fms, PVA=%.2fms, Grad=%.2fms, Time=%.2fms, Total=%.2fms\n",
               opt->iter_num_, traj_gen_time, smoothness_time, pva_cost_time, grad_time, time_cost_time, total_callback_time);
        fflush(stdout);  // Immediate output for debug info
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
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
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

    int size = swarm_trajs_->size();
    
    // 안전한 크기 계산: 실제 swarm_trajs 크기와 formation_size 중 작은 값 사용
    int effective_size = std::min(static_cast<int>(swarm_trajs_->size()), formation_size_);
    
    // formation_size_보다 작으면 아직 모든 드론의 궤적이 준비되지 않은 상태
    if (effective_size < formation_size_) {
      RCLCPP_DEBUG(node_->get_logger(), "Not all swarm trajectories ready: %d/%d", effective_size, formation_size_);
      return false;
    }
    
    size = effective_size;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    
    // Bounds check for drone_id_
    if (drone_id_ < 0 || drone_id_ >= formation_size_) {
        RCLCPP_WARN(node_->get_logger(), "drone_id_ %d out of bounds (formation_size: %d), using default position", drone_id_, formation_size_);
        return false;  // Still return false but with warning instead of error
    }
    
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (size_t id = 0; id < size; id++)
    {
      if (id == drone_id_)
        continue;
      
      // 이중 안전장치: size는 이미 안전한 값이지만 추가 확인
      if (id >= swarm_trajs_->size()) {
        RCLCPP_WARN(node_->get_logger(), "Swarm trajectory index %zu out of bounds (size: %zu)", id, swarm_trajs_->size());
        continue;
      }
      
      // Additional bounds check for formation arrays
      if (id >= formation_size_) {
        RCLCPP_WARN(node_->get_logger(), "Formation index %zu out of bounds (formation_size: %d)", id, formation_size_);
        continue;
      }
      
      // Check if trajectory is properly initialized (drone_id should be valid)
      if (swarm_trajs_->at(id).drone_id < 0) {
        RCLCPP_DEBUG(node_->get_logger(), "Skipping uninitialized trajectory at index %zu", id);
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
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    // Verify all positions are valid before updating graph
    bool valid_positions = true;
    for (size_t i = 0; i < swarm_graph_pos.size(); i++) {
        if (!std::isfinite(swarm_graph_pos[i].norm())) {
            RCLCPP_WARN(node_->get_logger(), "Invalid position at index %zu", i);
            valid_positions = false;
            break;
        }
    }
    
    if (!valid_positions) {
        return false;
    }
    
    // Ensure swarm_graph_pos has the correct size before updating
    if (static_cast<int>(swarm_graph_pos.size()) != formation_size_) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Swarm graph position vector size (%zu) doesn't match formation_size (%d)", 
                    swarm_graph_pos.size(), formation_size_);
        return false;
    }
    
    if (!swarm_graph_->updateGraph(swarm_graph_pos)) {
        RCLCPP_DEBUG(node_->get_logger(), "Failed to update swarm graph");
        return false;
    }

    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);

    if (similarity_error > 0)
    {
      ret = true;
      costp = wei_formation_ * similarity_error;
      vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);

      gradp = wei_formation_ * swarm_grad[drone_id_];

      for (size_t id = 0; id < size; id++)
      {
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    debug_similarity_ = similarity_error;
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

    // Convert to 2D position (ignore z-axis)
    Eigen::Vector3d p_2d = p;
    p_2d(2) = 0.0;  // Set z-axis to 0

    double dist;
    grid_map_->evaluateEDT(p_2d, dist);

    double dist_err = obs_clearance_ - dist;

    if (dist_err > 0)
    {
      ret = true;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateFirstGrad(p_2d, dist_grad);
      
              // 2D gradient (z-axis set to 0)
        dist_grad(2) = 0.0;
      
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

    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    double pt_time = t_now_ + t;

    // Early exit if no swarm trajectories
    if (swarm_trajs_->empty()) {
        return false;
    }
    
    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      // Additional bounds check for safety
      if (id >= swarm_trajs_->size()) {
        RCLCPP_WARN(node_->get_logger(), "Swarm trajectory index %zu out of bounds in swarmGradCostP", id);
        break;
      }
      
      // Check if trajectory is properly initialized and not our own drone
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }
      
      // Additional safety check for trajectory validity
      if (swarm_trajs_->at(id).duration <= 0.0) {
        RCLCPP_DEBUG(node_->get_logger(), "Skipping invalid trajectory duration at index %zu", id);
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      Eigen::Vector3d swarm_p, swarm_v;
      
      // Pre-calculate time difference to avoid repeated calculation
      double time_diff = pt_time - traj_i_satrt_time;
      
      if (time_diff < swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(time_diff);
        swarm_v = swarm_trajs_->at(id).traj.getVel(time_diff);
      }
      else
      {
        double exceed_time = time_diff - swarm_trajs_->at(id).duration;
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      
              // 2D distance calculation (ignore z-axis)
        Eigen::Vector3d dist_vec = p - swarm_p;
        dist_vec(2) = 0.0;  // Set z-axis distance to 0
        
        // 2D Euclidean distance calculation
      double dist2 = dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1);
      double dist2_err = CLEARANCE2 - dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;
        costp += wei_swarm_ * dist2_err3;
        
                  // 2D gradient (z-axis is 0)
          Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) *
                                  Eigen::Vector3d(dist_vec(0), dist_vec(1), 0.0);
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_ > dist2)
      {
        min_ellip_dist2_ = dist2;
      }
    }
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    // 2D velocity calculation (ignore z-axis)
    Eigen::Vector3d v_2d = v;
    v_2d(2) = 0.0;
    
    double vpen = v_2d.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v_2d;  // z-axis gradient is 0
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    // 2D acceleration calculation (ignore z-axis)
    Eigen::Vector3d a_2d = a;
    a_2d(2) = 0.0;
    
    double apen = a_2d.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a_2d;  // z-axis gradient is 0
      costa = wei_feas_ * apen * apen * apen;
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

  void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           vector<Eigen::Vector3d> &simple_path,
                                           Eigen::MatrixXd &ctl_points,
                                           poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos = finState.col(0);

    simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos);

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
    RCLCPP_INFO(node_->get_logger(), "Obstacle avoidance: %s", enable_obstacles_ ? "enabled" : "disabled");
    
    node_->declare_parameter("enable_debug_logs", false);
    node_->get_parameter("enable_debug_logs", enable_debug_logs_);
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

    node_->declare_parameter("optimization/obstacle_clearance", 0.1);
    node_->get_parameter("optimization/obstacle_clearance", obs_clearance_);
    node_->declare_parameter("optimization/swarm_clearance", 0.5);
    node_->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node_->declare_parameter("optimization/max_vel", 1.0);
    node_->get_parameter("optimization/max_vel", max_vel_);
    node_->declare_parameter("optimization/max_acc", 1.0);
    node_->get_parameter("optimization/max_acc", max_acc_);

    std::string log_file_path;
    node_->declare_parameter("optimization/log_file_path", "/home/suv/ws/ros_ws/optimizer.log");
    node_->get_parameter("optimization/log_file_path", log_file_path);
    initLogFile(log_file_path);

    swarm_graph_.reset(new SwarmGraph);
    
    // Set initial formation based on optimizer_params.yaml
    int initial_formation_type = 2; // Default to REGULAR_SQUARE
    node_->declare_parameter("optimization/formation_type", initial_formation_type);
    node_->get_parameter("optimization/formation_type", initial_formation_type);
    
    RCLCPP_INFO(node_->get_logger(), "Setting initial formation type: %d", initial_formation_type);
    setDesiredFormation(initial_formation_type);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;
    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector2i(400, 200));  // 2D for rover
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
      
      RCLCPP_INFO(node_->get_logger(), "Formation set with %d positions for optimizer (drone %d)", 
                  static_cast<int>(adjusted_formation.size()), drone_id_);
      
      // Print current formation details
      RCLCPP_INFO(node_->get_logger(), "=== CURRENT FORMATION CONFIGURATION ===");
      RCLCPP_INFO(node_->get_logger(), "Formation Size: %d", formation_size_);
      RCLCPP_INFO(node_->get_logger(), "Drone ID: %d", drone_id_);
      RCLCPP_INFO(node_->get_logger(), "Formation Positions:");
      for (size_t i = 0; i < adjusted_formation.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "  Drone %zu: [%.3f, %.3f, %.3f]", 
                    i, adjusted_formation[i].x(), adjusted_formation[i].y(), adjusted_formation[i].z());
      }
      RCLCPP_INFO(node_->get_logger(), "=====================================");
    } else {
      use_formation_ = false;
      formation_size_ = 0;
      RCLCPP_WARN(node_->get_logger(), "Failed to set formation - swarm_graph not initialized or empty positions");
    }
  }
}
