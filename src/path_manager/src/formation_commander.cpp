#include <rclcpp/rclcpp.hpp>
#include "path_manager/msg/formation_command.hpp"
#include "path_manager/msg/poly_traj.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include "path_optimizer/poly_traj_utils.hpp"

using namespace std::chrono_literals;

/**
 * @brief Lightweight FormationCommander - only publishes formation commands
 *
 * Task assignment is now handled in a distributed manner by each drone's replan_fsm
 */
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

        RCLCPP_INFO(this->get_logger(), "FormationCommander started - num_drones: %d, distance_threshold: %.1f",
                   num_drones_, distance_threshold_);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 10),
            qos_profile
        );

        formation_cmd_pub_ = this->create_publisher<path_manager::msg::FormationCommand>(
            "formation_command", sensor_qos
        );

        // Subscribe to trajectories for distance checking
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

        // Publish first command after 2 seconds, then switch to distance-based
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
    }

    void checkDistanceAndPublish() {
        if (!have_initial_command_ || command_count_ >= 8) {
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
        if (command_count_ >= 7) {
            RCLCPP_INFO(this->get_logger(), "Formation command sequence completed. Stopping distance check timer.");
            if (distance_check_timer_) {
                distance_check_timer_->cancel();
            }
            return;
        }

        path_manager::msg::FormationCommand msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        // Define formation sequences
        //
        // Available formation types:
        //   - "line_first"          : Line formation at 83° with offset (each drone has different waypoint)
        //   - "line_first": Line formation at 83° without offset (all drones share waypoints, local optimizer maintains formation)
        //   - "line_second"         : Line formation at -8.63° with offset
        //   - "line_second": Line formation at -8.63° without offset
        //   - "triangle"            : Triangle formation
        //   - "square"              : Square formation
        //
        // Use no_offset variants when:
        //   - Complex curved paths where offset causes formation drift
        //   - Want to rely purely on local formation constraint
        //   - Optimization sometimes "gives up" on formation and follows global path alone
        //
        // Use regular (with offset) variants when:
        //   - Straight or simple curved paths
        //   - Want explicit waypoint separation for each drone
        //
        switch (command_count_) {
            case 0:
                msg.formation_center.x = -8.75;
                msg.formation_center.y = -57.0;
                msg.formation_center.z = 0.0;
                msg.formation_type = "line_first";  // Use "line_first" if formation drifts on curves
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

                msg.waypoints.resize(2);
                msg.waypoints[0].x = 22.88972; msg.waypoints[0].y = -71.87321; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 30.23262; msg.waypoints[1].y = -74.74562; msg.waypoints[1].z = 0.0;
                break;

            case 2:
                msg.formation_center.x = 32.45039;
                msg.formation_center.y = -80.23940;
                msg.formation_center.z = 0.0;
                msg.formation_type = "triangle";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(3);
                msg.waypoints[0].x = 32.45039; msg.waypoints[0].y = -80.23940; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 32.90863; msg.waypoints[1].y = -87.44061; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 32.47298; msg.waypoints[2].y = -93.42377; msg.waypoints[2].z = 0.0;
                break;

            case 3:
                // Stage 1: triangle -> square formation change + initial path
                msg.formation_center.x = 32.47;
                msg.formation_center.y = -93.42;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(3);
                msg.waypoints[0].x = 32.47298; msg.waypoints[0].y = -93.42377; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 27.85286; msg.waypoints[1].y = -116.42038; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 27.01373; msg.waypoints[2].y = -122.2; msg.waypoints[2].z = 0.0;
                break;

            case 4:
                // Stage 2: square formation maintained, continue long straight path
                msg.formation_center.x = 42.09200;
                msg.formation_center.y = -130.01768;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(4);
                msg.waypoints[0].x = 27.48601; msg.waypoints[0].y = -127.08222; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 38.37347; msg.waypoints[1].y = -129.20889; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 42.09200; msg.waypoints[2].y = -130.01768; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 100.63176; msg.waypoints[3].y = -138.94522; msg.waypoints[3].z = 0.0;
                break;

            case 5:
                // Combined long path: square -> line_second formation with curved path, circle, and straight segments
                msg.formation_center.x = 89.56;
                msg.formation_center.y = -26.97;
                msg.formation_center.z = 0.0;
                msg.formation_type = "line_first";
                msg.formation_scale = 1.5;

                msg.waypoints.resize(17);
                // Initial curved path
                msg.waypoints[0].x = 94.61884; msg.waypoints[0].y = -137.36130; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 106.19094; msg.waypoints[1].y = -138.85164; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = 109.88909; msg.waypoints[2].y = -136.74628; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 112.11637; msg.waypoints[3].y = -133.03310; msg.waypoints[3].z = 0.0;
                msg.waypoints[4].x = 114.09413; msg.waypoints[4].y = -122.49115; msg.waypoints[4].z = 0.0;
                // First half of circle path
                msg.waypoints[5].x = 116.19815; msg.waypoints[5].y = -106.08804; msg.waypoints[5].z = 0.0;
                msg.waypoints[6].x = 121.31105; msg.waypoints[6].y = -104.47885; msg.waypoints[6].z = 0.0;
                msg.waypoints[7].x = 129.03363; msg.waypoints[7].y = -101.68507; msg.waypoints[7].z = 0.0;
                msg.waypoints[8].x = 133.40306; msg.waypoints[8].y = -96.61195; msg.waypoints[8].z = 0.0;
                msg.waypoints[9].x = 137.13039; msg.waypoints[9].y = -88.31696; msg.waypoints[9].z = 0.0;
                // Second half of circle path
                msg.waypoints[10].x = 134.43517; msg.waypoints[10].y = -77.65039; msg.waypoints[10].z = 0.0;
                msg.waypoints[11].x = 128.59427; msg.waypoints[11].y = -72.46378; msg.waypoints[11].z = 0.0;
                msg.waypoints[12].x = 121.21349; msg.waypoints[12].y = -68.27953; msg.waypoints[12].z = 0.0;
                msg.waypoints[13].x = 123.41044; msg.waypoints[13].y = -52.36917; msg.waypoints[13].z = 0.0;
                msg.waypoints[14].x = 124.92587; msg.waypoints[14].y = -37.03749; msg.waypoints[14].z = 0.0;
                msg.waypoints[15].x = 122.65699; msg.waypoints[15].y = -32.65216; msg.waypoints[15].z = 0.0;
                // Long straight path with obstacles
                msg.waypoints[16].x = 89.56; msg.waypoints[16].y = -26.97; msg.waypoints[16].z = 0.0;
                break;
            case 6:
                // Final: line_second -> square formation change
                msg.formation_center.x = 3.48957;
                msg.formation_center.y = -14.59468;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 2.0;

                msg.waypoints.resize(4);
                msg.waypoints[0].x = 3.48957; msg.waypoints[0].y = -14.59468; msg.waypoints[0].z = 0.0;
                msg.waypoints[1].x = 0.23684; msg.waypoints[1].y = -12.25636; msg.waypoints[1].z = 0.0;
                msg.waypoints[2].x = -0.85307; msg.waypoints[2].y = -8.07603; msg.waypoints[2].z = 0.0;
                msg.waypoints[3].x = 0.04395; msg.waypoints[3].y = 2.61689; msg.waypoints[3].z = 0.0;
                break;
        }

        // Update current formation center
        if (!msg.waypoints.empty()) {
            const auto &last_wp = msg.waypoints.back();
            current_formation_center_ = Eigen::Vector3d(last_wp.x, last_wp.y, last_wp.z);
        } else {
            current_formation_center_ = Eigen::Vector3d(
                msg.formation_center.x, msg.formation_center.y, msg.formation_center.z
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
