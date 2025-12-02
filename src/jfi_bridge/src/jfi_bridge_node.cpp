#include "jfi_bridge/jfi_bridge_node.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <iomanip>

JfiBridgeNode::JfiBridgeNode()
: Node("jfi_bridge_node"),
  last_sent_formation_cmd_sequence_(-1),
  poly_traj_serializer_(),
  formation_cmd_serializer_()
{
  /* -------- 1. Parameter handling ------------------------------------- */
  declare_parameter<int>("system_id", 1);
  system_id_ = static_cast<uint8_t>(get_parameter("system_id").as_int());

  RCLCPP_INFO(get_logger(), "Starting JFi Bridge Node with system_id=%d", system_id_);

  /* -------- 2. Setup topic names -------------------------------------- */
  std::string sid = std::to_string(system_id_);
  const std::string topic_prefix = "/V" + sid;

  /* -------- 3. Publishers --------------------------------------------- */
  // Use RELIABLE QoS to match jfi_comm (which uses default RELIABLE)
  auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // Use BEST_EFFORT for path_manager topics (original sensor_data QoS)
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_best_effort = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  // To jfi_comm (ROS -> Serial) - RELIABLE to match jfi_comm
  pub_to_jfi_ = create_publisher<jfi_comm::msg::SwarmComm>(
      "jfi_comm/in/packet", qos_reliable);

  // From serial to ROS - BEST_EFFORT for path_manager
  pub_poly_traj_ = create_publisher<path_manager::msg::PolyTraj>(
      topic_prefix + "/j_fi/broadcast_traj_recv", qos_best_effort);

  pub_formation_cmd_ = create_publisher<path_manager::msg::FormationCommand>(
      topic_prefix + "/formation_command", qos_best_effort);

  /* -------- 4. Subscribers -------------------------------------------- */

  // From path_manager (ROS -> Serial): PolyTraj - BEST_EFFORT
  sub_poly_traj_ = create_subscription<path_manager::msg::PolyTraj>(
      topic_prefix + "/planning/broadcast_traj_send", qos_best_effort,
      std::bind(&JfiBridgeNode::polyTrajToSerialCallback, this, std::placeholders::_1));

  // From path_manager (ROS -> Serial): FormationCommand (Commander only) - BEST_EFFORT
  if (system_id_ == 1) {
    sub_formation_cmd_ = create_subscription<path_manager::msg::FormationCommand>(
        "formation_command", qos_best_effort,
        std::bind(&JfiBridgeNode::formationCommandToSerialCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Commander mode: Subscribed to 'formation_command'");
  } else {
    RCLCPP_INFO(get_logger(), "Follower mode: Not subscribing to formation_command");
  }

  // From jfi_comm (Serial -> ROS) - RELIABLE to match jfi_comm
  sub_from_jfi_ = create_subscription<jfi_comm::msg::SwarmComm>(
      "jfi_comm/out/packet", qos_reliable,
      std::bind(&JfiBridgeNode::swarmCommFromSerialCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "JFi Bridge Node initialized successfully");
}

template<typename T>
std::vector<uint8_t> JfiBridgeNode::serializeMessage(const T& msg)
{
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;

  serializer.serialize_message(&msg, &serialized_msg);

  auto& rcl_serialized = serialized_msg.get_rcl_serialized_message();
  std::vector<uint8_t> payload(rcl_serialized.buffer_length);
  std::memcpy(payload.data(), rcl_serialized.buffer, rcl_serialized.buffer_length);

  return payload;
}

template<typename T>
T JfiBridgeNode::deserializeMessage(const std::vector<uint8_t>& data)
{
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;

  // Reserve buffer and copy data (following jfi_comm evaluator_node pattern)
  serialized_msg.reserve(data.size());
  auto& rcl_serialized = serialized_msg.get_rcl_serialized_message();
  rcl_serialized.buffer_length = data.size();
  std::memcpy(rcl_serialized.buffer, data.data(), data.size());

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}

void JfiBridgeNode::polyTrajToSerialCallback(const path_manager::msg::PolyTraj::SharedPtr msg)
{
  try {
    RCLCPP_INFO(get_logger(), "[TX] PolyTraj BEFORE serialize: drone_id=%d, traj_id=%d, coef_x=%zu, coef_y=%zu, coef_z=%zu",
                msg->drone_id, msg->traj_id, msg->coef_x.size(), msg->coef_y.size(), msg->coef_z.size());

    auto payload = serializeMessage(*msg);

    RCLCPP_INFO(get_logger(), "[TX] PolyTraj AFTER serialize: payload_size=%zu bytes", payload.size());

    auto swarm_msg = std::make_unique<jfi_comm::msg::SwarmComm>();
    swarm_msg->header.stamp = this->get_clock()->now();
    swarm_msg->src_sysid = system_id_;
    swarm_msg->tid = TID_POLY_TRAJ;
    swarm_msg->payload = payload;

    pub_to_jfi_->publish(std::move(swarm_msg));

    RCLCPP_INFO(get_logger(), "[TX] Sent PolyTraj via serial: drone_id=%d, traj_id=%d, size=%zu bytes",
                msg->drone_id, msg->traj_id, payload.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "[TX] Failed to serialize PolyTraj: %s", e.what());
  }
}

void JfiBridgeNode::formationCommandToSerialCallback(const path_manager::msg::FormationCommand::SharedPtr msg)
{
  // Prevent duplicate transmissions
  if (msg->sequence == last_sent_formation_cmd_sequence_) {
    return;
  }

  try {
    auto payload = serializeMessage(*msg);

    auto swarm_msg = std::make_unique<jfi_comm::msg::SwarmComm>();
    swarm_msg->header.stamp = this->get_clock()->now();
    swarm_msg->src_sysid = system_id_;
    swarm_msg->tid = TID_FORMATION_COMMAND;
    swarm_msg->payload = payload;

    pub_to_jfi_->publish(std::move(swarm_msg));
    last_sent_formation_cmd_sequence_ = msg->sequence;

    RCLCPP_INFO(get_logger(),
                "Sent FormationCommand via serial: seq=%d, mission=%s->%s, formation=%s, waypoints=%zu, size=%zu",
                msg->sequence, msg->current_mission_id.c_str(), msg->next_mission_id.c_str(),
                msg->formation_type.c_str(), msg->waypoints.size(), payload.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to serialize FormationCommand: %s", e.what());
  }
}

void JfiBridgeNode::swarmCommFromSerialCallback(const jfi_comm::msg::SwarmComm::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[RX] SwarmComm received: TID=%d, src_sysid=%d, seq=%u, payload_size=%zu",
               msg->tid, msg->src_sysid, msg->seq, msg->payload.size());

  // Log first 32 bytes of payload for debugging
  std::stringstream hex_dump;
  size_t dump_size = std::min(size_t(32), msg->payload.size());
  for (size_t i = 0; i < dump_size; ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02x ", msg->payload[i]);
    hex_dump << buf;
  }
  RCLCPP_INFO(get_logger(), "[RX] Payload hex (first %zu bytes): %s", dump_size, hex_dump.str().c_str());

  try {
    switch (msg->tid) {
      case TID_POLY_TRAJ: {
        RCLCPP_INFO(get_logger(), "[RX] PolyTraj BEFORE deserialize: payload_size=%zu", msg->payload.size());

        auto poly_traj = deserializeMessage<path_manager::msg::PolyTraj>(msg->payload);

        RCLCPP_INFO(get_logger(),
                    "[RX] PolyTraj AFTER deserialize: drone_id=%d, traj_id=%d, coef_x=%zu, coef_y=%zu, coef_z=%zu",
                    poly_traj.drone_id, poly_traj.traj_id,
                    poly_traj.coef_x.size(), poly_traj.coef_y.size(), poly_traj.coef_z.size());

        pub_poly_traj_->publish(poly_traj);

        RCLCPP_INFO(get_logger(),
                    "[RX] Published PolyTraj: drone_id=%d, traj_id=%d, coef_x=%zu, coef_y=%zu",
                    poly_traj.drone_id, poly_traj.traj_id,
                    poly_traj.coef_x.size(), poly_traj.coef_y.size());
        break;
      }

      case TID_FORMATION_COMMAND: {
        RCLCPP_INFO(get_logger(), "[RX] FormationCommand BEFORE deserialize: payload_size=%zu", msg->payload.size());

        auto formation_cmd = deserializeMessage<path_manager::msg::FormationCommand>(msg->payload);

        RCLCPP_INFO(get_logger(),
                    "[RX] FormationCommand AFTER deserialize: seq=%d, mission=%s->%s, formation=%s, waypoints=%zu",
                    formation_cmd.sequence,
                    formation_cmd.current_mission_id.c_str(),
                    formation_cmd.next_mission_id.c_str(),
                    formation_cmd.formation_type.c_str(),
                    formation_cmd.waypoints.size());

        pub_formation_cmd_->publish(formation_cmd);

        RCLCPP_INFO(get_logger(), "[RX] Published FormationCommand: seq=%d", formation_cmd.sequence);
        break;
      }

      default:
        RCLCPP_WARN(get_logger(), "[RX] Received message with unknown TID: %d", msg->tid);
        break;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "[RX] Failed to deserialize message (TID=%d, payload_size=%zu): %s",
                 msg->tid, msg->payload.size(), e.what());
  }
}
