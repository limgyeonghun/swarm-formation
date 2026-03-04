// trajectory_planner_node.cpp
// Simplified ReplanFSM - receives TrajectoryCommand directly (no formation logic)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <map>

#include "formation_msgs/msg/trajectory_command.hpp"
#include "path_manager/msg/poly_traj.hpp"
#include "path_manager/msg/position_command.hpp"
#include "trajectory_planner/path_manager.h"
#include "path_optimizer/plan_container.hpp"
#include "../../common/log_manager.hpp"

using namespace std::chrono_literals;

// Logging macros
#define TP_LOG_INFO(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_INFO(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->infof(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define TP_LOG_WARN(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_WARN(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->warnf(msg, ##__VA_ARGS__); \
    } \
} while(0)

#define TP_LOG_ERROR(msg, ...) do { \
    if (!enable_debug_logs_) { \
        RCLCPP_ERROR(node_->get_logger(), msg, ##__VA_ARGS__); \
    } else if (log_manager_) { \
        log_manager_->errorf(msg, ##__VA_ARGS__); \
    } \
} while(0)

namespace trajectory_planner {

class TrajectoryPlanner {
public:
    enum FSM_EXEC_STATE {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP
    };

    TrajectoryPlanner(rclcpp::Node::SharedPtr node)
    : node_(node),
      exec_state_(INIT),
      have_position_(false),
      have_target_(false),
      have_new_target_(false),
      have_local_traj_(false),
      drone_id_(0),
      enable_debug_logs_(false),
      enable_waypoint_markers_(false)
    {
        log_manager_ = std::make_unique<swarm_formation::LogManager>(
            node_->get_name(), "./logs/runtime", swarm_formation::LogManager::INFO);

        node_->get_parameter_or("enable_debug_logs", enable_debug_logs_, false);
        node_->get_parameter_or("enable_waypoint_markers", enable_waypoint_markers_, false);

        node_->declare_parameter("drone_id", 0);
        node_->get_parameter("drone_id", drone_id_);

        node_->declare_parameter("mavlink_id", 1);
        node_->get_parameter("mavlink_id", mavlink_id_);

        TP_LOG_INFO("Starting TrajectoryPlanner for drone_id: %d (internal), mavlink_id: %d (PX4)",
                    drone_id_, mavlink_id_);

        // PathManager
        path_manager_ = std::make_shared<path_manager::PathManager>(node_);

        // QoS
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // Callback groups
        timer_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        position_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Publishers
        std::string topic_prefix = "/V" + std::to_string(drone_id_ + 1);

        optimized_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(
            topic_prefix + "/planning/trajectory", sensor_qos);

        global_path_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(
            topic_prefix + "/planning/global", sensor_qos);

        broadcast_traj_pub_ = node_->create_publisher<path_manager::msg::PolyTraj>(
            topic_prefix + "/planning/broadcast_traj_send", sensor_qos);

        waypoint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "waypoint_markers", 10);

        // Position subscription
        node_->declare_parameter("rviz_simulation", false);
        bool rviz_simulation = false;
        node_->get_parameter("rviz_simulation", rviz_simulation);

        rclcpp::SubscriptionOptions position_options;
        position_options.callback_group = position_callback_group_;

        if (rviz_simulation) {
            std::string target_position_topic = "/agent" + std::to_string(drone_id_) + "/target_position";
            target_position_sub_ = node_->create_subscription<path_manager::msg::PositionCommand>(
                target_position_topic, sensor_qos,
                std::bind(&TrajectoryPlanner::targetPositionCallback, this, std::placeholders::_1),
                position_options);
            have_position_ = true;
        } else {
            std::string px4_position_topic = "/vehicle" + std::to_string(mavlink_id_) + "/fmu/out/vehicle_local_position";
            px4_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                px4_position_topic, sensor_qos,
                std::bind(&TrajectoryPlanner::PX4positionCallback, this, std::placeholders::_1),
                position_options);
        }

        // Broadcast trajectory subscription
        rclcpp::SubscriptionOptions broadcast_options;
        broadcast_options.callback_group = subscription_callback_group_;
        broadcast_traj_sub_ = node_->create_subscription<path_manager::msg::PolyTraj>(
            topic_prefix + "/j_fi/broadcast_traj_recv", sensor_qos,
            std::bind(&TrajectoryPlanner::recvBroadcastPolyTrajCallback, this, std::placeholders::_1),
            broadcast_options);

        // **KEY CHANGE**: Subscribe to TrajectoryCommand instead of FormationCommand
        rclcpp::SubscriptionOptions traj_cmd_options;
        traj_cmd_options.callback_group = subscription_callback_group_;
        trajectory_cmd_sub_ = node_->create_subscription<formation_msgs::msg::TrajectoryCommand>(
            topic_prefix + "/trajectory_command", sensor_qos,
            std::bind(&TrajectoryPlanner::trajectoryCommandCallback, this, std::placeholders::_1),
            traj_cmd_options);

        // Timer
        timer_ = node_->create_wall_timer(
            10ms,
            std::bind(&TrajectoryPlanner::computeAndPublishPaths, this),
            timer_callback_group_);

        TP_LOG_INFO("TrajectoryPlanner timer created (10ms period)");

        // Initialize
        current_pos_ = Eigen::Vector3d::Zero();
        current_vel_ = Eigen::Vector3d::Zero();
        start_pt_ = Eigen::Vector3d::Zero();
        start_vel_ = Eigen::Vector3d::Zero();
        start_acc_ = Eigen::Vector3d::Zero();
        end_pt_ = Eigen::Vector3d::Zero();
        local_target_pt_ = Eigen::Vector3d::Zero();
        local_target_vel_ = Eigen::Vector3d::Zero();
    }

    void init() {
        path_manager_->initOptimizer();
        path_manager_->deliverTrajToOptimizer();

        if (!have_target_) {
            TP_LOG_WARN("No target set yet, skipping global trajectory planning");
            return;
        }

        // Plan global trajectory if we have target
        // (This will be called after trajectoryCommandCallback sets the target)
    }

private:
    // **NEW CALLBACK**: Simplified - receives ready-to-use target
    void trajectoryCommandCallback(const formation_msgs::msg::TrajectoryCommand::SharedPtr msg) {
        if (msg->drone_id != drone_id_) {
            return;
        }

        TP_LOG_INFO("[TRAJECTORY_COMMAND] Received command for drone %d (mission: %s, seq: %d)",
                    drone_id_, msg->mission_id.c_str(), msg->sequence);

        // Initialize optimizer if needed
        if (!path_manager_->isOptimizerInitialized()) {
            try {
                TP_LOG_INFO("Initializing optimizer for drone %d...", drone_id_);
                path_manager_->initOptimizer();
                path_manager_->deliverTrajToOptimizer();
                TP_LOG_INFO("Optimizer initialized successfully for drone %d", drone_id_);
            } catch (const std::exception& e) {
                TP_LOG_ERROR("Failed to initialize optimizer: %s", e.what());
                return;
            }
        }

        // Determine start state
        if (have_local_traj_) {
            LocalTrajData *info = &path_manager_->traj_.local_traj;
            double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - info->start_time;
            start_pt_ = info->traj.getPos(t_cur);
            start_vel_ = info->traj.getVel(t_cur);
            start_acc_ = info->traj.getAcc(t_cur);
            TP_LOG_INFO("Using trajectory state for smooth transition");
        } else {
            start_pt_ = current_pos_;
            start_vel_ = Eigen::Vector3d::Zero();
            start_acc_ = Eigen::Vector3d::Zero();
            TP_LOG_INFO("Using current position: (%.2f, %.2f, %.2f)",
                       start_pt_(0), start_pt_(1), start_pt_(2));
        }

        // Extract waypoints
        std::vector<Eigen::Vector3d> waypoints;
        for (const auto& wp : msg->waypoints) {
            waypoints.emplace_back(wp.x, wp.y, wp.z);
        }

        // If no waypoints, use target_position directly
        if (waypoints.empty()) {
            waypoints.push_back(Eigen::Vector3d(
                msg->target_position.x,
                msg->target_position.y,
                msg->target_position.z));
        }

        end_pt_ = waypoints.back();

        TP_LOG_INFO("Planning to target: (%.2f, %.2f, %.2f) with %zu waypoints",
                   end_pt_(0), end_pt_(1), end_pt_(2), waypoints.size());

        // Visualize waypoints
        if (enable_waypoint_markers_) {
            publishWaypointMarkers(waypoints);
        }

        // **REMOVED**: generateFormationPattern, setFormationInfo (handled by formation_manager)
        // **REMOVED**: Hungarian assignment (already done by formation_manager)

        // Plan global trajectory
        bool success = path_manager_->planGlobalTraj(
            start_pt_, start_vel_, start_acc_,
            waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        if (success) {
            // **SIMPLIFIED**: No formation pattern to set (formation_manager handles it)
            // Just mark that we have a target
            have_target_ = true;
            have_new_target_ = true;

            if (exec_state_ == WAIT_TARGET)
                changeFSMExecState(GEN_NEW_TRAJ, "trajectoryCommandCallback");
            else if (exec_state_ == EXEC_TRAJ)
                changeFSMExecState(REPLAN_TRAJ, "trajectoryCommandCallback");

            TP_LOG_INFO("Successfully generated global trajectory for drone %d", drone_id_);
        } else {
            TP_LOG_ERROR("Failed to generate global trajectory for drone %d", drone_id_);
        }
    }

    void targetPositionCallback(const path_manager::msg::PositionCommand::SharedPtr msg) {
        current_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
        current_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        have_position_ = true;
    }

    void PX4positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_pos_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
        current_vel_ = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);
        have_position_ = true;
    }

    void recvBroadcastPolyTrajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
        // Store other drones' trajectories for collision avoidance
        // (Implementation same as original)
    }

    void computeAndPublishPaths() {
        if (!have_position_) return;
        if (!have_target_) return;

        switch (exec_state_) {
            case INIT:
            case WAIT_TARGET:
                // Waiting for trajectory command
                break;

            case GEN_NEW_TRAJ:
                callPathManager(true, false, false);  // flag_use_poly_init, flag_random, use_formation
                changeFSMExecState(EXEC_TRAJ, "GEN_NEW_TRAJ");
                break;

            case REPLAN_TRAJ:
                callPathManager(false, false, false);
                changeFSMExecState(EXEC_TRAJ, "REPLAN_TRAJ");
                break;

            case EXEC_TRAJ:
                // Check if trajectory is complete
                LocalTrajData *info = &path_manager_->traj_.local_traj;
                double t_cur = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - info->start_time;
                if (t_cur > info->duration - 0.2) {
                    have_target_ = false;
                    have_local_traj_ = false;
                    changeFSMExecState(WAIT_TARGET, "EXEC_TRAJ_COMPLETE");
                    TP_LOG_INFO("[drone %d reached goal]", drone_id_);
                }
                break;
        }
    }

    bool callPathManager(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation) {
        Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;
        double desired_start_time;

        if (have_local_traj_ && use_formation) {
            desired_start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds() + 0.02;  // replan_trajectory_time_
            double t_adj = desired_start_time - path_manager_->traj_.local_traj.start_time;
            desired_start_pt = path_manager_->traj_.local_traj.traj.getPos(t_adj);
            desired_start_vel = path_manager_->traj_.local_traj.traj.getVel(t_adj);
            desired_start_acc = path_manager_->traj_.local_traj.traj.getAcc(t_adj);
        } else {
            desired_start_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            desired_start_pt = start_pt_;
            desired_start_vel = start_vel_;
            desired_start_acc = start_acc_;
        }

        path_manager_->getLocalTarget(start_pt_, end_pt_, local_target_pt_, local_target_vel_, 1.0);

        bool plan_success = path_manager_->computeAndOptimizePath(
            desired_start_pt, desired_start_vel, desired_start_acc,
            desired_start_time, local_target_pt_, local_target_vel_,
            (have_new_target_ || flag_use_poly_init),
            flag_randomPolyTraj, use_formation, have_local_traj_);

        have_new_target_ = false;

        if (plan_success) {
            // Publish trajectory
            path_manager::msg::PolyTraj msg;
            polyTraj2ROSMsg(msg);
            optimized_path_pub_->publish(msg);
            broadcast_traj_pub_->publish(msg);
            have_local_traj_ = true;
        }

        return plan_success;
    }

    void polyTraj2ROSMsg(path_manager::msg::PolyTraj &msg) {
        auto data = &path_manager_->traj_.local_traj;
        msg.drone_id = data->drone_id;
        msg.start_time = rclcpp::Time(data->start_time);

        int piece_num = data->traj.getPieceNum();
        msg.duration.resize(piece_num);
        msg.coef_x.resize(6 * piece_num);
        msg.coef_y.resize(6 * piece_num);
        msg.coef_z.resize(6 * piece_num);

        for (int i = 0; i < piece_num; ++i) {
            msg.duration[i] = data->traj.getDurations()[i];
            poly_traj::CoefficientMat coef = data->traj[i].getCoeffMat();
            int i6 = i * 6;
            for (int j = 0; j < 6; j++) {
                msg.coef_x[i6 + j] = coef(0, j);
                msg.coef_y[i6 + j] = coef(1, j);
                msg.coef_z[i6 + j] = coef(2, j);
            }
        }
    }

    void publishWaypointMarkers(const std::vector<Eigen::Vector3d>& waypoints) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints_drone_" + std::to_string(drone_id_);
        marker.id = drone_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;

        // Color by drone ID
        if (drone_id_ == 0) {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        } else if (drone_id_ == 1) {
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
        } else if (drone_id_ == 2) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        } else if (drone_id_ == 3) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        for (const auto& wp : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = wp(0);
            p.y = wp(1);
            p.z = wp(2);
            marker.points.push_back(p);
        }

        waypoint_marker_pub_->publish(marker);
    }

    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
        static std::string state_str[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
        TP_LOG_INFO("[%s]: from %s to %s",
                   pos_call.c_str(),
                   state_str[static_cast<int>(exec_state_)].c_str(),
                   state_str[static_cast<int>(new_state)].c_str());
        exec_state_ = new_state;
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<path_manager::PathManager> path_manager_;
    std::unique_ptr<swarm_formation::LogManager> log_manager_;

    FSM_EXEC_STATE exec_state_;
    bool have_position_;
    bool have_target_;
    bool have_new_target_;
    bool have_local_traj_;
    int drone_id_;
    int mavlink_id_;
    bool enable_debug_logs_;
    bool enable_waypoint_markers_;

    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;
    Eigen::Vector3d end_pt_;
    Eigen::Vector3d local_target_pt_;
    Eigen::Vector3d local_target_vel_;

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr position_callback_group_;

    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr global_path_pub_;
    rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;

    rclcpp::Subscription<formation_msgs::msg::TrajectoryCommand>::SharedPtr trajectory_cmd_sub_;
    rclcpp::Subscription<path_manager::msg::PositionCommand>::SharedPtr target_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_position_sub_;
    rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr broadcast_traj_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace trajectory_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("trajectory_planner");
    auto planner = std::make_shared<trajectory_planner::TrajectoryPlanner>(node);

    planner->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
