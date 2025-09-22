#include <rclcpp/rclcpp.hpp>
#include "path_manager/msg/formation_command.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FormationCommander : public rclcpp::Node
{
public:
    FormationCommander() 
    : Node("formation_commander"), command_count_(0)
    {
        // QoS
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 10), 
            qos_profile
        );

        formation_cmd_pub_ = this->create_publisher<path_manager::msg::FormationCommand>(
            "formation_command", sensor_qos
        );

        initial_timer_ = this->create_wall_timer(
            2000ms,
            [this]() {
                publishFormationCommand();
                initial_timer_->cancel();

                periodic_timer_ = this->create_wall_timer(
                    25000ms,
                    std::bind(&FormationCommander::publishFormationCommand, this)
                );

                RCLCPP_INFO(this->get_logger(),
                    "Switched to periodic publishing: every 25s");
            }
        );

        RCLCPP_INFO(
            this->get_logger(), 
            "FormationCommander started - will publish first command in ~2s, then every 25s"
        );
    }

private:
    void publishFormationCommand()
    {
        path_manager::msg::FormationCommand msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        switch (command_count_ % 4) {
            case 0:
                msg.formation_center.x = 80.0;
                msg.formation_center.y = -12.5;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 3.0;
                break;
            case 1:
                msg.formation_center.x = 80.0;
                msg.formation_center.y = -12.5;
                msg.formation_center.z = 0.0;
                msg.formation_type = "triangle";
                msg.formation_scale = 2.5;
                break;
            case 2:
                msg.formation_center.x = 85.0;
                msg.formation_center.y = -12.5;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 3.0;
                break;
            case 3:
                msg.formation_center.x = 80.0;
                msg.formation_center.y = -12.5;
                msg.formation_center.z = 0.0;
                msg.formation_type = "square";
                msg.formation_scale = 2.0;
                break;
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
    rclcpp::TimerBase::SharedPtr initial_timer_;
    rclcpp::TimerBase::SharedPtr periodic_timer_;
    int command_count_;
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
