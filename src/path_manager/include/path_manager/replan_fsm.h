#ifndef REPLAN_FSM_H
#define REPLAN_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include "path_manager/msg/poly_traj.hpp"
#include "path_manager/path_manager.h"
#include "path_optimizer/plan_container.hpp"

namespace path_manager {

class ReplanFSM : public rclcpp::Node {
public:
    enum FSM_EXEC_STATE {
        INIT,
        WAIT_POSITION,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP,
        SEQUENTIAL_START
    };

    ReplanFSM();
    ~ReplanFSM() = default;
    
    void init();
    void computeAndPublishPaths();
    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
    void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg);
    void globalTraj2ROSMsg(path_manager::msg::PolyTraj &msg);

private:
    bool callPathManager(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation);
    bool planFromGlobalTraj(int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);

    std::shared_ptr<PathManager> path_manager_;

    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr global_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    FSM_EXEC_STATE exec_state_;
    int continously_called_times_;
    bool have_position_;
    bool have_target_;
    bool have_new_target_;
    bool have_local_traj_;
    bool have_recv_pre_agent_;
    bool flag_relan_astar_;
    int drone_id_;
    double replan_thresh_;
    double no_replan_thresh_;
    double replan_trajectory_time_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;
    Eigen::Vector3d end_pt_;
    Eigen::Vector3d local_target_pt_;
    Eigen::Vector3d local_target_vel_;
    double t_to_target_;
    double current_time_;
    double last_start_time_;
};

}  // namespace path_manager

#endif  // REPLAN_FSM_H