#include "path_visualization/path_visualization.h"
using namespace std::chrono_literals;

PathVisualization::PathVisualization()
    : Node("path_visualization"),
      is_optimizer_initialized_(false),
      first_call_(true),
      iniState_(3, 3),
      finState_(3, 3)
{
  // Load and validate obstacle parameters
  this->declare_parameter("obstacles", std::vector<double>{});
  std::vector<double> obstacle_params;
  this->get_parameter("obstacles", obstacle_params);
  if (obstacle_params.empty() || obstacle_params.size() % 3 != 0) {
    RCLCPP_WARN(this->get_logger(), "No or invalid obstacles provided, using default values.");
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

  // Setup publishers and subscriptions with QoS settings

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_markers", sensor_qos);
  min_jerk_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("min_jerk_trajectory", sensor_qos);
  control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("control_points", sensor_qos);
  optimized_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("opt_trajectory", sensor_qos);
  intermediate_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("intermediate_trajectory", sensor_qos);
  intermediate_control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("intermediate_control_points", sensor_qos);
  cost_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cost_text", sensor_qos);

  position_sub_ = this->create_subscription<px4_msgs::msg::Monitoring>(
      "/vehicle1/fmu/out/monitoring", sensor_qos,
      std::bind(&PathVisualization::positionCallback, this, std::placeholders::_1));

  // Initialize grid map
  Eigen::Vector3d map_size(50.0, 20.0, 5.0);
  double resolution = 0.1;
  grid_map_ = std::make_shared<GridMap>();
  grid_map_->initMap(map_size, resolution);

  Eigen::Vector3i voxel_num = grid_map_->getVoxelNum();
  int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
  std::vector<double> static_map(buffer_size, 0.0);
  grid_map_->setStaticMap(static_map);

  // Process obstacles: set occupancy and inflate
  for (const auto &obs : obstacle_centers_) {
    Eigen::Vector3i idx;
    grid_map_->posToIndex(obs, idx);
    grid_map_->setOccupancy(idx, 1.0);
    grid_map_->inflatePoint(idx, 5);
    RCLCPP_INFO(this->get_logger(), "Obstacle at: (%f, %f, %f), index: (%d, %d, %d)",
                obs.x(), obs.y(), obs.z(), idx.x(), idx.y(), idx.z());
  }
  grid_map_->updateESDF3d();
  publishObstacles();

  // Setup trajectory optimizer
  poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
  poly_traj_opt_->setEnvironment(grid_map_);
  poly_traj_opt_->setDroneId(0);

  // Initialize start and end states
  iniState_.col(0) << -4.0, -4.0, 0.5;
  iniState_.col(1) << 1.0, 0.0, 0.0;
  iniState_.col(2) << 0.0, 0.0, 0.0;

  finState_.col(0) << 10.0, 9.5, 0.5;
  finState_.col(1) << 0.0, 0.0, 0.0;
  finState_.col(2) << 0.0, 0.0, 0.0;

  start_pt_ = iniState_.col(0);
  end_pt_ = finState_.col(0);

  timer_ = this->create_wall_timer(20000ms, std::bind(&PathVisualization::computeAndPublishPaths, this));
}

void PathVisualization::initOptimizer()
{
  if (!is_optimizer_initialized_) {
    poly_traj_opt_->setParam(shared_from_this());
    is_optimizer_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "PolyTrajOptimizer parameters set.");
  }
}

void PathVisualization::positionCallback(const px4_msgs::msg::Monitoring::SharedPtr msg)
{
  start_pt_ = Eigen::Vector3d(msg->pos_x, msg->pos_y, 0.5);
  iniState_.col(0) = start_pt_;
}

void PathVisualization::getLocalTarget(double planning_horizen, const Eigen::Vector3d &start_pt,
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

visualization_msgs::msg::Marker PathVisualization::createMarker(const std::string &ns, int id, int type,
                                                                  double scale, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  return marker;
}

void PathVisualization::publishControlPoints(const Eigen::MatrixXd &control_points, int id, float r, float g, float b)
{
  auto marker = createMarker("control_points", id, visualization_msgs::msg::Marker::POINTS, 0.2, r, g, b, 1.0);
  for (int i = 0; i < control_points.cols(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = control_points(0, i);
    p.y = control_points(1, i);
    p.z = control_points(2, i);
    marker.points.push_back(p);
  }
  control_points_pub_->publish(marker);
}

void PathVisualization::computeAndPublishPaths()
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
    RCLCPP_INFO(this->get_logger(), "Calling astarWithMinTraj for global path...");
    poly_traj_opt_->astarWithMinTraj(iniState_, finState_, simple_path, control_points, init_mjo);

    if (simple_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "A* failed to find a global path! Check map, start/end points, or obstacles.");
      return;
    }
    simple_path_ = simple_path;
    publishPath(simple_path_, 1, 1.0, 0.0, 0.0, 0.3, marker_pub_);

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
  RCLCPP_INFO(this->get_logger(), "dist_to_target: %f, start_pt: (%f, %f, %f), local_target_pos: (%f, %f, %f)",
              dist_to_target, start_pt_.x(), start_pt_.y(), start_pt_.z(),
              local_target_pos.x(), local_target_pos.y(), local_target_pos.z());
  int piece_nums = std::max(2, static_cast<int>(dist_to_target / 2.0));
  RCLCPP_INFO(this->get_logger(), "piece_nums: %d", piece_nums);
  if (piece_nums < 2) {
    RCLCPP_WARN(this->get_logger(), "piece_nums is too small (%d), setting to 2", piece_nums);
    piece_nums = 2;
  }
  Eigen::MatrixXd innerPs(3, piece_nums - 1);
  Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_target / piece_nums);
  RCLCPP_INFO(this->get_logger(), "piece_dur_vec size: %d", piece_dur_vec.size());

  double t = 0.0;
  double t_step = t_to_target / piece_nums;
  for (int i = 0; i < piece_nums - 1; ++i) {
    t = std::min(t + t_step, global_traj_.duration);
    innerPs.col(i) = global_traj_.traj.getPos(t);
  }

  poly_traj::MinJerkOpt local_mjo;
  local_mjo.reset(headState, tailState, piece_nums);
  local_mjo.generate(innerPs, piece_dur_vec);

  // RCLCPP_INFO(this->get_logger(), "Sleeping for 0.5 seconds before minco...");
  // rclcpp::sleep_for(std::chrono::milliseconds(500));
  // RCLCPP_INFO(this->get_logger(), "Sleep finished, proceeding with minco...");

  // Visualize local trajectory
  poly_traj::Trajectory local_traj = local_mjo.getTraj();
  std::vector<Eigen::Vector3d> local_path;
  double total_time = local_traj.getTotalDuration();
  const double dt = 0.1;
  for (double t = 0.0; t <= total_time + dt; t += dt)
    local_path.push_back(local_traj.getPos(t));
  publishPath(local_path, 2, 0.0, 1.0, 0.0, 0.3, min_jerk_traj_pub_);

  // Optimize Trajectory
  int cps_num_prePiece = poly_traj_opt_->get_cps_num_prePiece_();
  Eigen::MatrixXd cstr_pts = local_mjo.getInitConstrainPoints(cps_num_prePiece);
  if (cstr_pts.cols() == 0) {
    RCLCPP_ERROR(this->get_logger(), "cstr_pts is empty! Cannot proceed with optimization.");
    return;
  }
  poly_traj_opt_->setControlPoints(cstr_pts);

  // RCLCPP_INFO(this->get_logger(), "Sleeping for 0.5 seconds before optimization...");
  // rclcpp::sleep_for(std::chrono::milliseconds(500));
  // RCLCPP_INFO(this->get_logger(), "Sleep finished, proceeding with optimization...");

  poly_traj_opt_->setIntermediateTrajCallback(
      [this](const poly_traj::Trajectory &traj, int iteration) {
        publishIntermediateTraj(traj, iteration);
      });
  poly_traj_opt_->setIntermediateControlPointsCallback(
      [this](const Eigen::MatrixXd &control_points, int iteration) {
        publishIntermediateControlPoints(control_points, iteration);
      });
  poly_traj_opt_->setCostCallback(
      [this](double cost, int iteration) {
        publishCostText(cost, iteration);
      });

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
    RCLCPP_INFO(this->get_logger(), "Optimization successful! Visualizing optimized trajectory...");
    poly_traj::Trajectory optimized_traj = poly_traj_opt_->getMinJerkOptPtr()->getTraj();
    std::vector<Eigen::Vector3d> optimized_path;
    double opt_total_time = optimized_traj.getTotalDuration();
    for (double t = 0.0; t <= opt_total_time + dt; t += dt)
      optimized_path.push_back(optimized_traj.getPos(t));
    RCLCPP_INFO(this->get_logger(), "optimized_path size: %zu, total_time: %f", optimized_path.size(), opt_total_time);
    publishPath(optimized_path, 3, 0.0, 0.0, 1.0, 1.0, optimized_traj_pub_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Optimization failed!");
  }
}

void PathVisualization::publishPath(const std::vector<Eigen::Vector3d> &path, int id, float r, float g, float b,
                                    float alpha, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  auto marker = createMarker("path", id, visualization_msgs::msg::Marker::LINE_STRIP, 0.05, r, g, b, alpha);
  for (const auto &pt : path) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    marker.points.push_back(p);
  }
  pub->publish(marker);
}

void PathVisualization::publishObstacles()
{
  for (size_t i = 0; i < obstacle_centers_.size(); ++i) {
    // Publish actual obstacle
    auto marker = createMarker("obstacle", i, visualization_msgs::msg::Marker::SPHERE, 1.0, 0.0, 1.0, 0.0, 0.5);
    marker.pose.position.x = obstacle_centers_[i].x();
    marker.pose.position.y = obstacle_centers_[i].y();
    marker.pose.position.z = obstacle_centers_[i].z();
    marker_pub_->publish(marker);

    // Publish virtual obstacle
    auto virtual_marker = createMarker("virtual_obstacle", i + obstacle_centers_.size(),
                                         visualization_msgs::msg::Marker::SPHERE, 3.0, 1.0, 0.0, 0.0, 0.3);
    virtual_marker.pose.position.x = obstacle_centers_[i].x();
    virtual_marker.pose.position.y = obstacle_centers_[i].y();
    virtual_marker.pose.position.z = obstacle_centers_[i].z();
    marker_pub_->publish(virtual_marker);
  }
}

void PathVisualization::publishPaths()
{
  publishPath(simple_path_, 1, 1.0, 0.0, 0.0, 0.3, marker_pub_);
  publishObstacles();
}

void PathVisualization::publishIntermediateTraj(const poly_traj::Trajectory &traj, int iteration)
{
  std::vector<Eigen::Vector3d> intermediate_path;
  double total_time = traj.getTotalDuration();
  const double dt = 0.1;
  for (double t = 0.0; t <= total_time + dt; t += dt)
    intermediate_path.push_back(traj.getPos(t));

  auto marker = createMarker("intermediate_path", 1, visualization_msgs::msg::Marker::LINE_STRIP, 0.03, 0.0, 0.0, 1.0, 1.0);
  for (const auto &pt : intermediate_path) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    marker.points.push_back(p);
  }
  intermediate_traj_pub_->publish(marker);
  RCLCPP_INFO(this->get_logger(), "Published intermediate trajectory at iteration %d", iteration);
}

void PathVisualization::publishIntermediateControlPoints(const Eigen::MatrixXd &control_points, int iteration)
{
  auto marker = createMarker("intermediate_control_points", iteration / 10,
                             visualization_msgs::msg::Marker::POINTS, 0.15, 0.0, 1.0, 1.0, 0.5);
  for (int i = 0; i < control_points.cols(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = control_points(0, i);
    p.y = control_points(1, i);
    p.z = control_points(2, i);
    marker.points.push_back(p);
  }
  intermediate_control_points_pub_->publish(marker);
  RCLCPP_INFO(this->get_logger(), "Published intermediate control points at iteration %d", iteration);
}

void PathVisualization::publishCostText(double cost, int iteration)
{
  auto marker = createMarker("cost_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 0.5, 1.0, 1.0, 1.0, 1.0);
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 2.0;
  marker.text = "Iteration: " + std::to_string(iteration) + ", Cost: " + std::to_string(cost);
  cost_text_pub_->publish(marker);
}
