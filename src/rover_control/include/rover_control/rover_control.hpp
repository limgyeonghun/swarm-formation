#ifndef ROVER_CONTROL__ROVER_CONTROL_HPP_
#define ROVER_CONTROL__ROVER_CONTROL_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <path_manager/msg/position_command.hpp>


using namespace std::chrono_literals;
using namespace std;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using path_manager::msg::PositionCommand;

class RoverControl : public rclcpp::Node
{
public:
    RoverControl();

private:
  int index_;          // Internal array index (0,1,2,3...)
  int mavlink_id_;     // MAVLink system ID (can be non-consecutive like 1,4,5,6)
  float offset_x_pt_;
  float offset_y_pt_;
  double target_idle_timeout_sec_;
  bool target_not_changing_{false};
  double arrival_distance_threshold_{false};
  bool have_target_{false};

  bool positions_equal(const PositionCommand& a, const PositionCommand& b) const;
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void disarm();
  void timer_cb();
  void pos_cb(const VehicleLocalPosition::SharedPtr msg) { curr_pos_ = *msg; }
  void status_cb(const VehicleStatus::SharedPtr msg) { status_ = *msg; }
  void target_cb(const PositionCommand::SharedPtr msg);

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<PositionCommand>::SharedPtr target_sub_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_target_update_time_;

  VehicleStatus status_;
  VehicleCommand cmd_;
  VehicleLocalPosition curr_pos_;
  PositionCommand target_pos_;
};

#endif  // ROVER_CONTROL__ROVER_CONTROL_HPP_