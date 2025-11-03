#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <path_manager/msg/poly_traj.hpp>
#include <path_manager/msg/position_command.hpp>
#include "path_manager/path_manager.h"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace path_manager {

class TrajServer : public rclcpp::Node {
public:
    TrajServer();

private:
    void trajCallback(const path_manager::msg::PolyTraj::SharedPtr msg);
    void publishPositionCommand();
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr traj_sub_;
    rclcpp::Publisher<path_manager::msg::PositionCommand>::SharedPtr pos_cmd_pub_;

    LocalTrajData local_traj_;
    int drone_id_;
    double n_seconds_ahead_;
    double rampup_duration_;
    double initial_lookahead_offset_;
    bool rviz_simulation_;
    bool have_local_traj_ = false;
    int last_traj_id_ = -1;
    double traj_update_time_ = 0.0;
};

TrajServer::TrajServer() : Node("traj_server") {
    declare_parameter("drone_id", 0);
    get_parameter("drone_id", drone_id_);
    RCLCPP_INFO(get_logger(), "Starting TrajServer for drone_id: %d", drone_id_);

    declare_parameter("rviz_simulation", false);
    get_parameter("rviz_simulation", rviz_simulation_);

    declare_parameter("fsm/n_seconds_ahead", -1.0);
    get_parameter("fsm/n_seconds_ahead", n_seconds_ahead_);

    declare_parameter("fsm/rampup_duration", 1.0);
    get_parameter("fsm/rampup_duration", rampup_duration_);

    declare_parameter("fsm/initial_lookahead_offset", 0.3);
    get_parameter("fsm/initial_lookahead_offset", initial_lookahead_offset_);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Internal agent topic (0,1,2,3...), not to be confused with external vehicle mavlink_id
    std::string pos_cmd_topic = "/agent" + std::to_string(drone_id_) + "/target_position";
    pos_cmd_pub_ = create_publisher<path_manager::msg::PositionCommand>(pos_cmd_topic, sensor_qos);

    traj_sub_ = create_subscription<path_manager::msg::PolyTraj>(
        "planning/trajectory", sensor_qos, std::bind(&TrajServer::trajCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&TrajServer::publishPositionCommand, this));
}

void TrajServer::trajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    if (msg->drone_id != drone_id_) return;

    // Track trajectory update for ramp-up logic
    if (msg->traj_id != last_traj_id_) {
        last_traj_id_ = msg->traj_id;
        traj_update_time_ = now().seconds();
    }

    local_traj_.drone_id = msg->drone_id;
    local_traj_.traj_id = msg->traj_id;
    local_traj_.start_time = rclcpp::Time(msg->start_time).seconds();

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
        int i6 = i * 6;
        cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                           msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
        cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                           msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
        cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                           msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];
        dura[i] = msg->duration[i];
    }

    local_traj_.traj = poly_traj::Trajectory(dura, cMats);
    local_traj_.duration = local_traj_.traj.getTotalDuration();
    have_local_traj_ = true;

    // RCLCPP_INFO(get_logger(), "Received new trajectory for drone %d, duration: %.2f", drone_id_, local_traj_.duration);
}

void TrajServer::publishPositionCommand() {
    if (!have_local_traj_) return;

    static rclcpp::Time prev_ros_time{};
    static std::chrono::steady_clock::time_point prev_wall_tp{};

    const rclcpp::Time now_ros = now();
    const auto now_wall = std::chrono::steady_clock::now();

    prev_ros_time = now_ros;
    prev_wall_tp = now_wall;

    double t_cur = now_ros.seconds() - local_traj_.start_time;
    t_cur = std::min(local_traj_.duration, t_cur);

    double t_ahead = (t_cur + n_seconds_ahead_);
    Eigen::Vector3d speed_cmd_pos = local_traj_.traj.getPos(t_ahead);
    Eigen::Vector3d speed_cmd_vel = local_traj_.traj.getVel(t_ahead);
    Eigen::Vector3d speed_cmd_acc = local_traj_.traj.getAcc(t_ahead);

    // Dynamic lookahead for direction to prevent oscillation at initial ramp-up
    double elapsed_since_update = now_ros.seconds() - traj_update_time_;
    double direction_lookahead = 0.0;

    if (elapsed_since_update < rampup_duration_) {
        // Gradually decrease the lookahead offset during ramp-up period
        double rampup_factor = 1.0 - (elapsed_since_update / rampup_duration_);
        direction_lookahead = initial_lookahead_offset_ * rampup_factor;
    }

    Eigen::Vector3d direction_cmd_pos = local_traj_.traj.getPos(t_cur + direction_lookahead);

    // Eigen::Vector3d jerk = local_traj_.traj.getJerk(t_ahead);

    double yaw = 0.0;
    double yaw_dot = 0.0;

    if (speed_cmd_vel.norm() > 1e-3) {
        yaw = std::atan2(speed_cmd_vel(1), speed_cmd_vel(0));
    }

    path_manager::msg::PositionCommand msg{};
    msg.header.stamp = now_ros;
    msg.header.frame_id = "odom";

    msg.position.x = speed_cmd_pos(0);
    msg.position.y = speed_cmd_pos(1);
    msg.position.z = speed_cmd_pos(2);

    msg.velocity.x = speed_cmd_vel(0);
    msg.velocity.y = speed_cmd_vel(1);
    msg.velocity.z = speed_cmd_vel(2);

    msg.acceleration.x = speed_cmd_acc(0);
    msg.acceleration.y = speed_cmd_acc(1);
    msg.acceleration.z = speed_cmd_acc(2);

    msg.jerk.x = NAN;
    msg.jerk.y = NAN;
    msg.jerk.z = NAN;

    // Lookahead point on trajectory (for direction calculation, prevents corner cutting)
    msg.lookahead_point.x = direction_cmd_pos.x();
    msg.lookahead_point.y = direction_cmd_pos.y();
    msg.lookahead_point.z = direction_cmd_pos.z();

    msg.yaw = yaw;
    msg.yaw_dot = yaw_dot;

    msg.trajectory_id = local_traj_.traj_id;

    // Check if we're at the end of the trajectory
    bool is_trajectory_completed = (t_cur >= local_traj_.duration - 0.1);  // 0.1s threshold before end
    msg.trajectory_flag = is_trajectory_completed ?
        path_manager::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED :
        path_manager::msg::PositionCommand::TRAJECTORY_STATUS_READY;

    pos_cmd_pub_->publish(msg);
}

}  // namespace path_manager

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_manager::TrajServer>());
    rclcpp::shutdown();
    return 0;
}