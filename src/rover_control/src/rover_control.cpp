#include <rover_control/rover_control.hpp>
#include <algorithm>
#include <cmath>

RoverControl::RoverControl() : Node("RoverControl"), index_(0), mavlink_id_(1), offset_x_pt_(0.0), offset_y_pt_(0.0)
{
    this->declare_parameter<int>("index", 0);
    this->get_parameter("index", index_);

    this->declare_parameter<int>("mavlink_id", 1);
    this->get_parameter("mavlink_id", mavlink_id_);

    this->declare_parameter<float>("start_point_x", 0.0);
    this->get_parameter("start_point_x", offset_x_pt_);

    this->declare_parameter<float>("start_point_y", 0.0);
    this->get_parameter("start_point_y", offset_y_pt_);

    this->declare_parameter<double>("target_idle_timeout_sec", 0.5);
    this->get_parameter("target_idle_timeout_sec", target_idle_timeout_sec_);

    this->declare_parameter<double>("arrival_distance_threshold", 0.5);
    this->get_parameter("arrival_distance_threshold", arrival_distance_threshold_);

    RCLCPP_INFO(this->get_logger(), "Index: %d (internal) | MAVLink ID: %d (vehicle) | target_timeout: %.2f  arrival_threshold: %.2f",
                index_, mavlink_id_, target_idle_timeout_sec_, arrival_distance_threshold_);

    std::string mavlink_str = std::to_string(mavlink_id_);
    const std::string topic_prefix_out = "/vehicle" + mavlink_str + "/fmu/out/";
    const std::string topic_prefix_in = "/vehicle" + mavlink_str + "/fmu/in/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 20), qos_profile);

    status_sub_ = this->create_subscription<VehicleStatus>(
        topic_prefix_out + "vehicle_status", qos, bind(&RoverControl::status_cb, this, std::placeholders::_1));
    position_sub_ = this->create_subscription<VehicleLocalPosition>(
        topic_prefix_out + "vehicle_local_position", qos, bind(&RoverControl::pos_cb, this, std::placeholders::_1));
    // target_sub_ = this->create_subscription<PositionCommand>(
    //     topic_prefix_in + "target_position", qos, bind(&RoverControl::target_cb, this, std::placeholders::_1));

    // Internal topic uses agent index (0,1,2,3...), external MAVLink uses vehicle mavlink_id
    std::string agent_id = std::to_string(index_);
    target_sub_ = this->create_subscription<PositionCommand>(
        "/agent" + agent_id + "/target_position", qos, bind(&RoverControl::target_cb, this, std::placeholders::_1));    
    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(topic_prefix_in + "trajectory_setpoint", qos);
    offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(topic_prefix_in + "offboard_control_mode", qos);
    command_pub_ = this->create_publisher<VehicleCommand>(topic_prefix_in + "vehicle_command", qos);

    timer_ = this->create_wall_timer(10ms, bind(&RoverControl::timer_cb, this));
}

bool RoverControl::positions_equal(const PositionCommand& a, const PositionCommand& b) const
{
    return (a.position.x == b.position.x) &&
           (a.position.y == b.position.y) &&
           (a.position.z == b.position.z) &&
           (a.velocity.x == b.velocity.x) &&
           (a.velocity.y == b.velocity.y) &&
           (a.velocity.z == b.velocity.z);
}

void RoverControl::target_cb(const PositionCommand::SharedPtr msg)
{
    PositionCommand new_target = *msg;

    if (have_target_) {
        if (!positions_equal(new_target, target_pos_)) {
            last_target_update_time_ = this->now();
        }
    } else {
        have_target_ = true;
        last_target_update_time_ = this->now();
    }
    target_pos_ = new_target;
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

    offboard_control_mode_pub_->publish(msg);
}

void RoverControl::publish_trajectory_setpoint()
{
    // RCLCPP_INFO(this->get_logger(), "status_.nav_state: %d", status_.nav_state);
    if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED && have_target_)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->now().nanoseconds();

        // Use lookahead_point from traj_server (calculated on trajectory from current position)
        msg.position[0] = target_pos_.lookahead_point.x - offset_x_pt_;
        msg.position[1] = target_pos_.lookahead_point.y - offset_y_pt_;
        // msg.position[2] = 0.0;

        // Use trajectory velocity for speed command
        msg.velocity[0] = target_pos_.position.x - offset_x_pt_;
        msg.velocity[1] = target_pos_.position.y - offset_y_pt_;
        msg.velocity[2] = target_pos_.velocity.z;

        trajectory_setpoint_pub_->publish(msg);
    }
    else 
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->now().nanoseconds();
        msg.position[0] = 0.0;
        msg.position[1] = 0.0;
        msg.position[2] = 0.0;

        msg.velocity[0] = NAN;
        msg.velocity[1] = NAN;
        msg.velocity[2] = NAN;

        trajectory_setpoint_pub_->publish(msg);
    }
}

void RoverControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = mavlink_id_;  // Use actual MAVLink system ID
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  command_pub_->publish(msg);
}

void RoverControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void RoverControl::timer_cb()
{
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    if (have_target_) {
        const double elapsed = (this->now() - last_target_update_time_).seconds();
        target_not_changing_ = (elapsed >= target_idle_timeout_sec_);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10,
        //     "target_not_changing_: %s (elapsed=%.3f s, threshold=%.3f s)",
        //     target_not_changing_ ? "true" : "false", elapsed, target_idle_timeout_sec_);

        if (target_not_changing_) {
            double adjusted_tx = target_pos_.position.x - offset_x_pt_;
            double adjusted_ty = target_pos_.position.y - offset_y_pt_;
            double dist = std::hypot(curr_pos_.x - adjusted_tx, curr_pos_.y - adjusted_ty);

            // Only disarm if: (1) trajectory is marked as completed AND (2) we're close to target AND (3) it's the final mission
            bool is_trajectory_completed = (target_pos_.trajectory_flag == PositionCommand::TRAJECTORY_STATUS_COMPLETED);
            bool is_final_mission = target_pos_.is_final_mission;

            if (is_trajectory_completed && is_final_mission && dist <= arrival_distance_threshold_ && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
                disarm();
                have_target_ = false;
                RCLCPP_INFO(this->get_logger(), "Arrived at FINAL target. Distance: %.2f m <= threshold %.2f m, trajectory_flag: COMPLETED, is_final_mission: TRUE",
                           dist, arrival_distance_threshold_);
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}