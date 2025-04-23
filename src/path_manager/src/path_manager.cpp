#include "path_manager/path_manager.h"

namespace path_manager
{

    PathManager::PathManager(const std::shared_ptr<rclcpp::Node> &node)
        : node_(node),
          max_vel_(1.5),
          max_acc_(8.0),
          poly_traj_piece_length_(2.0),
          planning_horizen_(7.5),
          is_optimizer_initialized_(false),
          first_call_(true)
    {
        int drone_id;
        node_->get_parameter("drone_id", drone_id);
        traj_.local_traj.drone_id = drone_id;

        node_->declare_parameter("manager/max_vel", 1.5);
        node_->declare_parameter("manager/max_acc", 8.0);
        node_->declare_parameter("manager/polyTraj_piece_length", 2.0);
        node_->declare_parameter("manager/planning_horizon", 7.5);
        node_->get_parameter("manager/max_vel", max_vel_);
        node_->get_parameter("manager/max_acc", max_acc_);
        node_->get_parameter("manager/polyTraj_piece_length", poly_traj_piece_length_);
        node_->get_parameter("manager/planning_horizon", planning_horizen_);

        Eigen::Vector3d map_size(70.0, 30.0, 3.0);
        double resolution = 0.1;
        grid_map_ = std::make_shared<GridMap>();
        grid_map_->initMap(map_size, resolution);

        Eigen::Vector3i voxel_num = grid_map_->getVoxelNum();
        int buffer_size = voxel_num(0) * voxel_num(1) * voxel_num(2);
        std::vector<double> static_map(buffer_size, 0.0);
        grid_map_->setStaticMap(static_map);

        node_->declare_parameter("obstacles", std::vector<double>{});
        std::vector<double> obstacle_params;
        node_->get_parameter("obstacles", obstacle_params);
        if (obstacle_params.empty() || obstacle_params.size() % 3 != 0)
        {
            obstacle_centers_ = {
                Eigen::Vector3d(-2.0, -2.25, 0.5),
                Eigen::Vector3d(1.0, 0.0, 0.5),
                Eigen::Vector3d(0.0, 1.0, 0.5)};
        }
        else
        {
            for (size_t i = 0; i < obstacle_params.size(); i += 3)
            {
                obstacle_centers_.emplace_back(obstacle_params[i], obstacle_params[i + 1], 0.5);
            }
        }

        for (const auto &obs : obstacle_centers_)
        {
            Eigen::Vector3i idx;
            grid_map_->posToIndex(obs, idx);
            grid_map_->setOccupancy(idx, 1.0);
            grid_map_->inflatePoint(idx, 5);
        }
        grid_map_->updateESDF3d();
    }

    void PathManager::initOptimizer()
    {
        if (is_optimizer_initialized_)
        {
            return;
        }
        
        poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>();
        poly_traj_opt_->setEnvironment(grid_map_);
        poly_traj_opt_->setDroneId(traj_.local_traj.drone_id);
        poly_traj_opt_->setParam(node_);

        is_optimizer_initialized_ = true;
    }

    bool PathManager::computeAndOptimizePath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const double trajectory_start_time, const Eigen::Vector3d &local_target_pt,
                                             const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
                                             const bool flag_randomPolyTraj, const bool use_formation, const bool have_local_traj)
    {
        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            return false;
        }

        double ts = poly_traj_piece_length_ / max_vel_;
        poly_traj::MinJerkOpt initMJO;
        if (!computeInitReferenceState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, ts, initMJO, flag_polyInit))
        {
            return false;
        }
        
        Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(poly_traj_opt_->get_cps_num_prePiece_());
        poly_traj_opt_->setControlPoints(cstr_pts);

        poly_traj::Trajectory initTraj = initMJO.getTraj();
        int PN = initTraj.getPieceNum();
        Eigen::MatrixXd all_pos = initTraj.getPositions();
        Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
        tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);

        bool flag_success = poly_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState, innerPts, initTraj.getDurations(), cstr_pts, true);
        if (!flag_success)
        {
            return false;
        }

        if (have_local_traj && use_formation)
        {
            double delta_replan_time = trajectory_start_time - rclcpp::Clock().now().seconds();
            if (delta_replan_time > 0)
            {
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_replan_time)));
            }
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
        }
        else
        {
            traj_.setLocalTraj(poly_traj_opt_->getMinJerkOptPtr()->getTraj(), rclcpp::Clock().now().seconds());
        }

        return true;
    }

    bool PathManager::computeInitReferenceState(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                                const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
                                                const Eigen::Vector3d &local_target_vel, const double &ts,
                                                poly_traj::MinJerkOpt &initMJO, const bool flag_polyInit)
    {
        if (first_call_ || flag_polyInit)
        {
            first_call_ = false;
            Eigen::Matrix3d headState, tailState;
            headState << start_pt, start_vel, start_acc;
            tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

            Eigen::MatrixXd ctl_points;
            poly_traj_opt_->astarWithMinTraj(headState, tailState, simple_path_, ctl_points, initMJO);
        }
        else
        {
            if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
            {
                return false;
            }

            double passed_t_on_lctraj = rclcpp::Clock().now().seconds() - traj_.local_traj.start_time;
            double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj; // local traj 끝나기까지 남은시간
            double t_to_lc_tgt = t_to_lc_end + (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);\
                // 새 궤적의 duration = 이전 local traj 기준 남은 시간(거리) + (새로 늘어난 목표지점)

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
                if (t < t_to_lc_end) // 현재 가리키는 지점(t) 가 이전로컬궤적안에 있다면 로컬궤적의 위치에서 가져오기
                {
                    innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
                }
                else if (t <= t_to_lc_tgt) // 현재 가리키는 지점(t)가 새로생긴 로컬궤적밖이라면 글로벌궤적에서 위치 가져오기
                {
                    double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
                    innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
                }
                else
                {
                    cout << "Should not happen! x_x 0x88" << endl;
                }

                t += piece_dur_vec(i + 1);
            }

            initMJO.reset(headState, tailState, piece_nums);
            initMJO.generate(innerPs, piece_dur_vec);
        }

        return true;
    }

    bool PathManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                                     const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
        poly_traj::MinJerkOpt globalMJO;
        Eigen::Matrix<double, 3, 3> headState, tailState;
        headState << start_pos, start_vel, start_acc;
        tailState << waypoints.back(), end_vel, end_acc;
        Eigen::MatrixXd innerPts;

        if (waypoints.size() > 1)
        {
            innerPts.resize(3, waypoints.size() - 1);
            for (int i = 0; i < waypoints.size() - 1; i++)
                innerPts.col(i) = waypoints[i];
        }
        else
        {
            if (innerPts.size() != 0)
            {
                cout << "innerPts.size() != 0" << endl;
            }
        }
        globalMJO.reset(headState, tailState, waypoints.size());

        double des_vel = max_vel_;
        Eigen::VectorXd time_vec(waypoints.size());
        int try_num = 0;
        do
        {
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                                       : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
            }
            globalMJO.generate(innerPts, time_vec);
            // cout << "try_num : " << try_num << endl;
            // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
            // cout << "time_vec : " << time_vec.transpose() << endl;

            des_vel /= 1.2;
            try_num++;
        } while (globalMJO.getTraj().getMaxVelRate() > max_vel_ && try_num <= 5);

        auto time_now = rclcpp::Clock().now().seconds();
        traj_.setGlobalTraj(globalMJO.getTraj(), time_now);

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
        // 이전 목표지점(시간) 가져오기 (traj_.global_traj.glb_t_of_lc_tgt)
        // 이전 목표지점부터 전역궤적 끝까지 탐색
        // t_step 만큼 증가하며 최적 지점 찾기
        // 시작지점부터 목표지점이 planning_horizen_ 보다 커지는 지점을 목표지점으로 설정
        // 그때의 시간 t를 glb_t_of_lc_tgt 에 저장 (이 코드에선 위치를 시간으로 표현)
        // 즉, 목표위치에 해당하는 전역 시간을 기록해둠 

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
            // 목표지점에서의 목표속력 계산
        }
    }

} // namespace path_manager