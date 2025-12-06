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
    bool rviz_simulation_;
    bool have_local_traj_ = false;
    int last_traj_id_ = -1;
    bool is_final_mission_ = false;
};

TrajServer::TrajServer() : Node("traj_server") {
    declare_parameter("drone_id", 0);
    get_parameter("drone_id", drone_id_);
    RCLCPP_INFO(get_logger(), "Starting TrajServer for drone_id: %d", drone_id_);

    declare_parameter("rviz_simulation", false);
    get_parameter("rviz_simulation", rviz_simulation_);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Internal agent topic (0,1,2,3...), not to be confused with external vehicle mavlink_id
    std::string pos_cmd_topic = "/agent" + std::to_string(drone_id_) + "/target_position";
    std::string topic_prefix = "/V" + std::to_string(drone_id_+1);
    pos_cmd_pub_ = create_publisher<path_manager::msg::PositionCommand>(pos_cmd_topic, sensor_qos);

    traj_sub_ = create_subscription<path_manager::msg::PolyTraj>(
        topic_prefix + "/planning/trajectory", sensor_qos, std::bind(&TrajServer::trajCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&TrajServer::publishPositionCommand, this));
}

void TrajServer::trajCallback(const path_manager::msg::PolyTraj::SharedPtr msg) {
    if (msg->drone_id != drone_id_) return;

    if (msg->traj_id != last_traj_id_) {
        last_traj_id_ = msg->traj_id;
    }
    is_final_mission_ = msg->is_final_mission;

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
}

void TrajServer::publishPositionCommand() {
    if (!have_local_traj_) return;

    const rclcpp::Time now_ros = now();
    double t_cur = now_ros.seconds() - local_traj_.start_time;

    Eigen::Vector3d pos, vel, acc;

    // Similar to Swarm-Formation logic
    if (t_cur < local_traj_.duration && t_cur >= 0.0) {
        // Normal execution: sample trajectory at current time
        pos = local_traj_.traj.getPos(t_cur);
        vel = local_traj_.traj.getVel(t_cur);
        acc = local_traj_.traj.getAcc(t_cur);
    } else if (t_cur >= local_traj_.duration) {
        // Trajectory finished: hover at end
        pos = local_traj_.traj.getPos(local_traj_.duration);
        vel.setZero();
        acc.setZero();
    } else {
        // t_cur < 0: trajectory not started yet, don't publish
        return;
    }

    // Calculate yaw from velocity
    double yaw = 0.0;
    double yaw_dot = 0.0;

    if (vel.norm() > 1e-3) {
        yaw = std::atan2(vel(1), vel(0));
    }

    // Publish position command
    path_manager::msg::PositionCommand msg{};
    msg.header.stamp = now_ros;
    msg.header.frame_id = "odom";

    msg.position.x = pos(0);
    msg.position.y = pos(1);
    msg.position.z = pos(2);

    msg.velocity.x = vel(0);
    msg.velocity.y = vel(1);
    msg.velocity.z = vel(2);

    msg.acceleration.x = acc(0);
    msg.acceleration.y = acc(1);
    msg.acceleration.z = acc(2);

    msg.jerk.x = NAN;
    msg.jerk.y = NAN;
    msg.jerk.z = NAN;

    // No separate lookahead - use same position
    msg.lookahead_point.x = pos.x();
    msg.lookahead_point.y = pos.y();
    msg.lookahead_point.z = pos.z();

    msg.yaw = yaw;
    msg.yaw_dot = yaw_dot;

    msg.trajectory_id = local_traj_.traj_id;

    // Check if we're at the end of the trajectory
    bool is_trajectory_completed = (t_cur >= local_traj_.duration - 0.1);  // 0.1s threshold before end
    msg.trajectory_flag = is_trajectory_completed ?
        path_manager::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED :
        path_manager::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    msg.is_final_mission = is_final_mission_;

    pos_cmd_pub_->publish(msg);
}

}  // namespace path_manager

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_manager::TrajServer>());
    rclcpp::shutdown();
    return 0;
}