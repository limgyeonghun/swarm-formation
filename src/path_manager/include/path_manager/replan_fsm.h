#ifndef REPLAN_FSM_H
#define REPLAN_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <map>
#include "path_manager/msg/poly_traj.hpp"
#include "path_manager/msg/formation_target.hpp"
#include "path_manager/msg/formation_command.hpp"
#include "path_manager/msg/position_command.hpp"
#include "path_manager/path_manager.h"
#include "path_optimizer/plan_container.hpp"
#include "swarm_graph/swarm_graph.hpp"
#include "../../common/log_manager.hpp"

// Conditional logging macros to avoid code duplication
#define FSM_LOG_INFO(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_INFO(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->infof(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_WARN(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_WARN(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->warnf(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_ERROR(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_ERROR(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->errorf(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define FSM_LOG_DEBUG(msg, ...) do { \
    if (enable_debug_logs_ && log_manager_) { \
        log_manager_->debugf(msg, ##__VA_ARGS__); \
    } \
} while(0)

namespace path_manager {

class ReplanFSM {
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

    ReplanFSM(rclcpp::Node::SharedPtr node);
    ~ReplanFSM() {};
    
    void init();
    void computeAndPublishPaths();
    void targetPositionCallback(const path_manager::msg::PositionCommand::SharedPtr msg);
    void PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
    void formationTargetCallback(const path_manager::msg::FormationTarget::SharedPtr msg);
    void formationCommandCallback(const path_manager::msg::FormationCommand::SharedPtr msg);
    void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg);
    void globalTraj2ROSMsg(path_manager::msg::PolyTraj &msg);
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

private:
    rclcpp::Node::SharedPtr node_;
    bool callPathManager(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation);
    bool planFromGlobalTraj(int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    bool isMapReady(const Eigen::Vector3d& start_pos);
    bool callEmergencyStop(const Eigen::Vector3d& stop_pos);
    
    // Formation manager functions
    void generateFormationTargets(const Eigen::Vector3d& center, const std::string& formation_type, double scale, const std::vector<Eigen::Vector3d>& waypoints = {});
    std::vector<Eigen::Vector3d> generateFormationPattern(const std::string& formation_type, int num_drones, double scale);
    void publishFormationTarget(const Eigen::Vector3d& target, const std::vector<Eigen::Vector3d>& waypoints = {}, bool formation_changed = false, const Eigen::Vector3d& formation_offset = Eigen::Vector3d::Zero());

    // Hungarian algorithm for optimal drone-target assignment
    std::vector<int> hungarianAssignment(const std::vector<Eigen::Vector3d>& current_positions,
                                         const std::vector<Eigen::Vector3d>& target_positions,
                                         const std::vector<Eigen::Vector3d>& waypoints);

    // Helper functions for line segment intersection checking
    bool segmentsIntersect2D(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                             const Eigen::Vector3d& p2, const Eigen::Vector3d& q2);
    int orientation(const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& r);
    bool onSegment(const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& r);

    std::shared_ptr<PathManager> path_manager_;

    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr global_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_pub_;
    rclcpp::Subscription<path_manager::msg::PositionCommand>::SharedPtr target_position_sub_;
    rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_position_sub_;
    rclcpp::Subscription<path_manager::msg::FormationTarget>::SharedPtr formation_target_sub_;
    rclcpp::Subscription<path_manager::msg::FormationCommand>::SharedPtr formation_cmd_sub_;
    rclcpp::Publisher<path_manager::msg::FormationTarget>::SharedPtr formation_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    FSM_EXEC_STATE exec_state_;
    int continously_called_times_;
    bool have_position_;
    bool have_target_;
    bool have_new_target_;
    bool have_local_traj_;
    bool have_recv_pre_agent_;
    bool flag_replan_astar_;
    int drone_id_;      // Internal index (0,1,2,3...)
    int mavlink_id_;    // MAVLink system ID for PX4 communication
    double replan_thresh_;
    double no_replan_thresh_;
    double replan_trajectory_time_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;
    Eigen::Vector3d end_pt_;
    Eigen::Vector3d local_target_pt_;
    Eigen::Vector3d local_target_vel_;
    Eigen::Vector3d offset_pt_;
    double t_to_target_;
    double current_time_;
    double last_start_time_;
    double n_seconds_ahead_;
    bool rviz_simulation_;
    bool flag_escape_emergency_;
    bool enable_debug_logs_;
    bool enable_waypoint_markers_;
    bool enable_global_trajectory_pub_;
    double hungarian_distance_weight_;   // Weight for distance cost in Hungarian assignment
    double hungarian_crossing_penalty_;  // Penalty weight for path crossings in Hungarian assignment

    // Formation manager variables
    int num_drones_;
    std::string current_formation_type_;
    std::string prev_formation_type_;  // Track previous formation type for change detection
    double current_formation_scale_;
    Eigen::Vector3d current_formation_center_;
    bool has_formation_command_;
    SwarmGraph::Ptr swarm_graph_;

    // Previous formation tracking for smooth transitions
    Eigen::Vector3d prev_end_pt_;  // Previous formation target endpoint
    Eigen::Vector3d prev_formation_offset_;  // Previous formation offset for this drone

    // Swarm position tracking for Hungarian assignment
    std::map<int, Eigen::Vector3d> swarm_positions_;  // drone_id -> current position
    std::mutex swarm_positions_mutex_;  // Thread-safe access

    std::unique_ptr<swarm_formation::LogManager> log_manager_;
};

}  // namespace path_manager

#endif  // REPLAN_FSM_H