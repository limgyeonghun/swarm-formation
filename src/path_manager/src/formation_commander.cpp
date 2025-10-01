#include <rclcpp/rclcpp.hpp>
#include "path_manager/msg/formation_command.hpp"
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include "path_optimizer/poly_traj_utils.hpp"

using namespace std::chrono_literals;

class FormationCommander : public rclcpp::Node
{
public:
    FormationCommander() 
    : Node("formation_commander"), 
      command_count_(0),
      current_formation_center_(0.0, 0.0, 0.0),
      have_initial_command_(false)
    {
        // Declare parameters
        this->declare_parameter("num_drones", 4);
        this->declare_parameter("distance_threshold", 3.0);
        
        // Get parameters
        num_drones_ = this->get_parameter("num_drones").as_int();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "FormationCommander started - num_drones: %d, distance_threshold: %.1f", num_drones_, distance_threshold_);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 10), 
            qos_profile
        );

        formation_cmd_pub_ = this->create_publisher<path_manager::msg::FormationCommand>(
            "formation_command", sensor_qos
        );

        for (int i = 0; i < num_drones_; ++i) {
            std::string topic_prefix = "/V" + std::to_string(i + 1);
            auto traj_sub = this->create_subscription<path_manager::msg::PolyTraj>(
                topic_prefix + "/planning/broadcast_traj_send", sensor_qos,
                [this, i](const path_manager::msg::PolyTraj::SharedPtr msg) {
                    this->trajectoryCallback(msg, i);
                }
            );
            trajectory_subs_.push_back(traj_sub);
        }

        drone_trajectories_.resize(num_drones_);

        // (First, operate by time (2s), then by distance)
        initial_timer_ = this->create_wall_timer(
            2000ms,
            [this]() {
                publishFormationCommand();
                initial_timer_->cancel();
                have_initial_command_ = true;

                distance_check_timer_ = this->create_wall_timer(
                    100ms,
                    std::bind(&FormationCommander::checkDistanceAndPublish, this)
                );

                RCLCPP_INFO(this->get_logger(),
                    "Switched to distance-based formation commands (threshold: %.1f m)", distance_threshold_);
            }
        );

        RCLCPP_INFO(
            this->get_logger(), 
            "FormationCommander started - will publish first command in ~2s, then distance-based"
        );
    }

private:
    struct TrajectoryData {
        poly_traj::Trajectory traj;
        double start_time;
        double duration;
        int drone_id;
        bool valid;
        
        TrajectoryData() : start_time(0.0), duration(0.0), drone_id(-1), valid(false) {}
    };

    void trajectoryCallback(const path_manager::msg::PolyTraj::SharedPtr msg, int drone_idx) {
        if (drone_idx < 0 || drone_idx >= num_drones_) return;
        
        if (msg->drone_id != drone_idx) {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring trajectory: expected drone_id=%d, got drone_id=%d", 
                        drone_idx, msg->drone_id);
            return;
        }
        
        auto& traj_data = drone_trajectories_[drone_idx];
        traj_data.drone_id = msg->drone_id;
        traj_data.start_time = rclcpp::Time(msg->start_time).seconds();
        traj_data.valid = true;

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

        traj_data.traj = poly_traj::Trajectory(dura, cMats);
        traj_data.duration = traj_data.traj.getTotalDuration();

        double now_time = this->now().seconds();
        double t_rel = now_time - traj_data.start_time;
        t_rel = std::min(traj_data.duration, std::max(0.0, t_rel));
        Eigen::Vector3d pos = traj_data.traj.getPos(t_rel);
    }

    void checkDistanceAndPublish() {
        if (!have_initial_command_ || command_count_ >= 4) {
            return;
        }
        
        Eigen::Vector3d current_swarm_center = getCurrentSwarmCenter();
        Eigen::Vector3d target_center = getTargetFormationCenter();
        double distance = (current_swarm_center - target_center).norm();
        
        if (distance <= distance_threshold_) {
            RCLCPP_INFO(this->get_logger(), 
                       "Distance threshold reached (%.2f <= %.2f), publishing next formation command", 
                       distance, distance_threshold_);
            publishFormationCommand();
        }
    }

    Eigen::Vector3d getCurrentSwarmCenter() {
        double current_time = this->now().seconds();
        Eigen::Vector3d center(0, 0, 0);
        int valid_drones = 0;
        
        for (int i = 0; i < num_drones_; ++i) {
            const auto& traj = drone_trajectories_[i];
            if (traj.valid) {
                double t_rel = current_time - traj.start_time;
                t_rel = std::min(traj.duration, std::max(0.0, t_rel));
                Eigen::Vector3d pos = traj.traj.getPos(t_rel);
                center += pos;
                valid_drones++;
            }
        }
        
        if (valid_drones > 0) {
            center /= valid_drones;
        }

        return center;
    }

    Eigen::Vector3d getTargetFormationCenter() {
        return current_formation_center_;
    }

    void publishFormationCommand()
    {
        if (command_count_ >= 3) {
            RCLCPP_INFO(this->get_logger(), "Formation command sequence completed. Stopping distance check timer.");
            if (distance_check_timer_) {
                distance_check_timer_->cancel();
            }
            return;
        }

        path_manager::msg::FormationCommand msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        switch (command_count_) {
            case 0:
                msg.formation_center.x = -8.75;
                msg.formation_center.y = -57.0;
                msg.formation_center.z = 0.0;
                msg.formation_type = "line_first";
                msg.formation_scale = 4.0;
                
                msg.waypoints.resize(5);
                msg.waypoints[0].x = -8.75; msg.waypoints[0].y = -57.0; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = -8.53; msg.waypoints[1].y = -63.3; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = -3.92; msg.waypoints[2].y = -66.04; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 2.11; msg.waypoints[3].y = -68.35; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 7.87; msg.waypoints[4].y = -68.99; msg.waypoints[4].z = 0.0;
            
                break;
            case 1:
                msg.formation_center.x = 22.88;
                msg.formation_center.y = -71.87;
                msg.formation_center.z = 0.0;
                msg.formation_type = "triangle";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(5);
                msg.waypoints[0].x = 22.88972; msg.waypoints[0].y = -71.87321; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 30.23262; msg.waypoints[1].y = -74.74562; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 32.45039; msg.waypoints[2].y = -80.23940; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 32.90863; msg.waypoints[3].y = -87.44061; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 32.47298; msg.waypoints[4].y = -93.42377; msg.waypoints[4].z = 0.0;

                break;
            case 2:
                msg.formation_center.x = 32.47;
                msg.formation_center.y = -93.42;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 2.0;
                
                msg.waypoints.resize(6);
                msg.waypoints[0].x = 32.47298; msg.waypoints[0].y = -93.42377; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 27.85286; msg.waypoints[1].y = -116.42038; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 29.01373; msg.waypoints[2].y = -121.56232; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 32.08601; msg.waypoints[3].y = -127.38222; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 38.37347; msg.waypoints[4].y = -129.20889; msg.waypoints[4].z = 0.0;
                msg.waypoints[5].x = 42.09200; msg.waypoints[5].y = -130.01768; msg.waypoints[5].z = 0.0;
                msg.waypoints[6].x = 100.63176; msg.waypoints[6].y = -138.94522; msg.waypoints[6].z = 0.0;
                break;
            case 3:
                msg.formation_center.x = 107;
                msg.formation_center.y = -138.76;
                msg.formation_center.z = 0.0;
                msg.formation_type = "line_second";
                msg.formation_scale = 5.0;
                
                msg.waypoints.resize(1);
                msg.waypoints[0].x = 107.0; msg.waypoints[0].y = -138.76; msg.waypoints[0].z = 0.0;
                break;
        }

        if (!msg.waypoints.empty()) {
            const auto &last_wp = msg.waypoints.back();
            current_formation_center_ = Eigen::Vector3d(
                last_wp.x,
                last_wp.y,
                last_wp.z
            );
        } else {
            current_formation_center_ = Eigen::Vector3d(
                msg.formation_center.x,
                msg.formation_center.y,
                msg.formation_center.z
            );
        }

        formation_cmd_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published formation command #%d: %s at (%.1f, %.1f, %.1f) scale %.1f",
            command_count_ + 1, 
            msg.formation_type.c_str(),
            msg.formation_center.x, 
            msg.formation_center.y, 
            msg.formation_center.z,
            msg.formation_scale
        );

        command_count_++;
    }

    rclcpp::Publisher<path_manager::msg::FormationCommand>::SharedPtr formation_cmd_pub_;
    std::vector<rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr> trajectory_subs_;
    rclcpp::TimerBase::SharedPtr initial_timer_;
    rclcpp::TimerBase::SharedPtr distance_check_timer_;

    int command_count_;
    int num_drones_;
    double distance_threshold_;
    Eigen::Vector3d current_formation_center_;
    bool have_initial_command_;
    std::vector<TrajectoryData> drone_trajectories_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FormationCommander>();
    RCLCPP_INFO(node->get_logger(), "FormationCommander node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
