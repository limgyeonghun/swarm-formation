#include "path_manager/path_manager.h"
using namespace std::chrono_literals;

PathManager::PathManager()
    : Node("path_manager"),
      is_optimizer_initialized_(false),
      first_call_(true),
      iniState_(3, 3),
      finState_(3, 3)
{
  // Load obstacle parameters
  this->declare_parameter("obstacles", std::vector<double>{});
  std::vector<double> obstacle_params;
  this->get_parameter("obstacles", obstacle_params);
  if (obstacle_params.empty() || obstacle_params.size() % 3 != 0) {
    obstacle_centers_ = {
        Eigen::Vector3d(-2.0, -2.25, 0.5),
        Eigen::Vector3d(1.0, 0.0, 0.5),
        Eigen::Vector3d(0.0, 1.0, 0.5),
        Eigen::Vector3d(3.0, 3.0, 0.5)};
  } else {
    for (size_t i = 0; i < obstacle_params.size(); i += 3) {
      obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], 0.5);
    }
  }

  optimized_path_pub_ = this->create_publisher<path_manager::msg::PolyTraj>("optimized_path", 10);

  // Initialize grid map
  Eigen::Vector3d map_size(50.0, 20.0, 5.0);
  double resolution = 0.1;
  grid_map_ = std::make_shared<GridMap>();
  grid_map_->initMap(map_size, resolution);

  Eigen::Vector3i voxel_num = grid_map_->getVoxelNum();
  int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
  std::vector<double> static_map(buffer_size, 0.0);
  grid_map_->setStaticMap(static_map);

  // Process obstacles
  for (const auto &obs : obstacle_centers_) {
    Eigen::Vector3i idx;
    grid_map_->posToIndex(obs, idx);
    RCLCPP_INFO(this->get_logger(), "Obstacle at (%.2f, %.2f, %.2f) -> index (%d, %d, %d)",
                obs.x(), obs.y(), obs.z(), idx.x(), idx.y(), idx.z());
    grid_map_->setOccupancy(idx, 1.0);
    grid_map_->inflatePoint(idx, 5);
  }
  grid_map_->updateESDF3d();
  

  // Setup trajectory optimizer
  poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
  poly_traj_opt_->setEnvironment(grid_map_);
  poly_traj_opt_->setDroneId(0);

  // Initialize states
  iniState_.col(0) << -4.0, -4.0, 0.5;
  iniState_.col(1) << 1.0, 0.0, 0.0;
  iniState_.col(2) << 0.0, 0.0, 0.0;

  finState_.col(0) << 10.0, 9.5, 0.5;
  finState_.col(1) << 0.0, 0.0, 0.0;
  finState_.col(2) << 0.0, 0.0, 0.0;

  start_pt_ = iniState_.col(0);
  end_pt_ = finState_.col(0);

  timer_ = this->create_wall_timer(20000ms, std::bind(&PathManager::computeAndPublishPaths, this));
}

void PathManager::initOptimizer()
{
  if (!is_optimizer_initialized_) {
    poly_traj_opt_->setParam(shared_from_this());
    is_optimizer_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "PolyTrajOptimizer parameters set.");
  }
}

void PathManager::polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg, const poly_traj::Trajectory &traj)
{
  msg.drone_id = 0;
  msg.traj_id = 1;
  msg.start_time = this->now();
  msg.order = 5;
  int piece_num = traj.getPieceNum();
  msg.duration.resize(piece_num);
  msg.coef_x.resize(6 * piece_num);
  msg.coef_y.resize(6 * piece_num);
  msg.coef_z.resize(6 * piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    msg.duration[i] = traj.getPiece(i).getDuration();
    poly_traj::CoefficientMat cMat = traj.getPiece(i).getCoeffMat();
    int offset = i * 6;
    for (int j = 0; j < 6; ++j)
    {
      msg.coef_x[offset + j] = cMat(0, j);
      msg.coef_y[offset + j] = cMat(1, j);
      msg.coef_z[offset + j] = cMat(2, j);
    }
  }
}


void PathManager::getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
                                 const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
                                 Eigen::Vector3d &local_target_vel, double &t_to_target)
{
  double t = global_traj_.glb_t_of_lc_tgt;
  const double t_step = planning_horizen / 20 / max_vel_;
  const double start_time = global_traj_.glb_t_of_lc_tgt;
  bool found = false;

  while (t < (global_traj_.global_start_time + global_traj_.duration)) {
    Eigen::Vector3d pos_t = global_traj_.traj.getPos(t - global_traj_.global_start_time);
    if ((pos_t - start_pt).norm() >= planning_horizen) {
      local_target_pos = pos_t;
      global_traj_.glb_t_of_lc_tgt = t;
      t_to_target = t - start_time;
      found = true;
      break;
    }
    t += t_step;
  }

  if (!found) {
    local_target_pos = global_end_pt;
    global_traj_.glb_t_of_lc_tgt = global_traj_.global_start_time + global_traj_.duration;
    t_to_target = global_traj_.duration - start_time;
  }

  if (t_to_target < 1e-6) {
    t_to_target = (start_pt - local_target_pos).norm() / max_vel_;
  }

  local_target_vel = ((global_end_pt - local_target_pos).norm() < (max_vel_ * max_vel_) / (2 * max_acc_))
                         ? Eigen::Vector3d::Zero()
                         : global_traj_.traj.getVel(t - global_traj_.global_start_time);
}

void PathManager::computeAndPublishPaths()
{
  initOptimizer();
  if ((start_pt_ - end_pt_).norm() < 0.1) {
    RCLCPP_INFO(this->get_logger(), "Reached goal! Stopping timer.");
    timer_->cancel();
    return;
  }

  if (first_call_) {
    std::vector<Eigen::Vector3d> simple_path;
    Eigen::MatrixXd control_points;
    poly_traj::MinJerkOpt init_mjo;
    poly_traj_opt_->astarWithMinTraj(iniState_, finState_, simple_path, control_points, init_mjo);
    if (simple_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "A* failed to find a global path!");
      return;
    }
    simple_path_ = simple_path;
    global_traj_.traj = init_mjo.getTraj();
    global_traj_.global_start_time = 0.0;
    global_traj_.duration = global_traj_.traj.getTotalDuration();
    global_traj_.glb_t_of_lc_tgt = 0.0;
    first_call_ = false;
  }

  const double planning_horizen = 5.0;
  Eigen::Vector3d local_target_pos, local_target_vel;
  double t_to_target;
  getLocalTarget(planning_horizen, start_pt_, end_pt_, local_target_pos, local_target_vel, t_to_target);

  Eigen::MatrixXd headState = iniState_;
  Eigen::MatrixXd tailState(3, 3);
  tailState.col(0) = local_target_pos;
  tailState.col(1) = local_target_vel;
  tailState.col(2) << 0.0, 0.0, 0.0;

  double dist_to_target = (start_pt_ - local_target_pos).norm();
  int piece_nums = std::max(2, static_cast<int>(dist_to_target / 2.0));
  Eigen::MatrixXd innerPs(3, piece_nums - 1);
  Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_target / piece_nums);

  double t = 0.0;
  double t_step = t_to_target / piece_nums;
  for (int i = 0; i < piece_nums - 1; ++i) {
    t = std::min(t + t_step, global_traj_.duration);
    innerPs.col(i) = global_traj_.traj.getPos(t);
  }

  poly_traj::MinJerkOpt local_mjo;
  local_mjo.reset(headState, tailState, piece_nums);
  local_mjo.generate(innerPs, piece_dur_vec);

  poly_traj::Trajectory local_traj = local_mjo.getTraj();
  std::vector<Eigen::Vector3d> local_path;
  double total_time = local_traj.getTotalDuration();
  const double dt = 0.1;
  for (double t = 0.0; t <= total_time + dt; t += dt)
    local_path.push_back(local_traj.getPos(t));

  int cps_num_prePiece = poly_traj_opt_->get_cps_num_prePiece_();
  Eigen::MatrixXd cstr_pts = local_mjo.getInitConstrainPoints(cps_num_prePiece);
  poly_traj_opt_->setControlPoints(cstr_pts);

  // not used
  poly_traj_opt_->setIntermediateTrajCallback(nullptr);
  poly_traj_opt_->setIntermediateControlPointsCallback(nullptr);
  poly_traj_opt_->setCostCallback(nullptr);

  poly_traj::Trajectory optTraj = local_mjo.getTraj();
  int PN = optTraj.getPieceNum();
  Eigen::MatrixXd all_pos = optTraj.getPositions();
  Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
  Eigen::Matrix<double, 3, 3> opt_headState, opt_tailState;
  opt_headState << optTraj.getJuncPos(0), optTraj.getJuncVel(0), optTraj.getJuncAcc(0);
  opt_tailState << optTraj.getJuncPos(PN), optTraj.getJuncVel(PN), optTraj.getJuncAcc(PN);

  bool flag_success = poly_traj_opt_->OptimizeTrajectory_lbfgs(opt_headState, opt_tailState,
                                                                 innerPts, optTraj.getDurations(),
                                                                 cstr_pts, false);

  if (flag_success) {
    poly_traj::Trajectory optimized_traj = poly_traj_opt_->getMinJerkOptPtr()->getTraj();
    path_manager::msg::PolyTraj opt_msg;
    polyTraj2ROSMsg(opt_msg, optimized_traj);
    optimized_path_pub_->publish(opt_msg);
    RCLCPP_INFO(this->get_logger(), "Published optimized path with %zu pieces", optimized_traj.getPieceNum());
  }
}