#include <rover_control/rover_control.hpp>
#include <algorithm>

RoverControl::RoverControl() : Node("RoverControl"), rover_id_(1)
{
    this->declare_parameter<int>("rover_id", 1);
    this->get_parameter("rover_id", rover_id_);

    std::string sid = std::to_string(rover_id_ + 1);
    const std::string topic_prefix_out = "/vehicle" + sid + "/fmu/out/";
    const std::string topic_prefix_in = "/vehicle" + sid + "/fmu/in/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

    status_sub_ = this->create_subscription<VehicleStatus>(
        topic_prefix_out + "vehicle_status", qos, bind(&RoverControl::status_cb, this, std::placeholders::_1));
    position_sub_ = this->create_subscription<VehicleLocalPosition>(
        topic_prefix_out + "vehicle_local_position", qos, bind(&RoverControl::pos_cb, this, std::placeholders::_1));
    // odom_sub_ = this->create_subscription<Odometry>(
    //     topic_prefix_in + "target_position", qos, bind(&RoverControl::target_cb, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<Odometry>(
        "vehicle" + sid + "/target_position", qos, bind(&RoverControl::target_cb, this, std::placeholders::_1));    
    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(topic_prefix_in + "trajectory_setpoint", qos);
    offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(topic_prefix_in + "offboard_control_mode", qos);

    timer_ = this->create_wall_timer(10ms, bind(&RoverControl::timer_cb, this));
}

void RoverControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};

    msg.timestamp = this->now().nanoseconds();
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.actuator = false;

    offboard_control_mode_pub_->publish(msg);
}

void RoverControl::publish_trajectory_setpoint()
{
    // RCLCPP_INFO(this->get_logger(), "status_.nav_state: %d", status_.nav_state);
    if (status_.nav_state == 14)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->now().nanoseconds();
        msg.position[0] = target_pos_.pose.pose.position.x;
        msg.position[1] = target_pos_.pose.pose.position.y - (1.5 * rover_id_);
        msg.position[2] = - target_pos_.pose.pose.position.z;

        // double average_velocity = std::sqrt(
            // std::pow(target_pos_.twist.twist.linear.x, 2) + std::pow(target_pos_.twist.twist.linear.y, 2));
        msg.velocity[0] = target_pos_.twist.twist.linear.x;
        msg.velocity[1] = target_pos_.twist.twist.linear.y;
        msg.velocity[2] = target_pos_.twist.twist.linear.z;

        // msg.acceleration[0] = target_pos_.twist.twist.linear.x;
        // msg.acceleration[1] = target_pos_.twist.twist.linear.y;

        trajectory_setpoint_pub_->publish(msg);
    }
}

void RoverControl::timer_cb()
{
    publish_offboard_control_mode();
    publish_trajectory_setpoint();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}