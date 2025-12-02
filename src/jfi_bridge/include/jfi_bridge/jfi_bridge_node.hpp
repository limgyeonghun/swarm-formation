#ifndef JFI_BRIDGE_NODE_HPP
#define JFI_BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <path_manager/msg/poly_traj.hpp>
#include <path_manager/msg/formation_command.hpp>
#include <jfi_comm/msg/swarm_comm.hpp>
#include <rclcpp/serialization.hpp>

/**
 * @class JfiBridgeNode
 * @brief Bridge between path_manager topics and jfi_comm SwarmComm interface
 *
 * This node converts between ROS2 messages (PolyTraj, FormationCommand) and
 * the generic SwarmComm payload format used by jfi_comm for serial transmission.
 */
class JfiBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Message type identifiers (TID) for different message types
   */
  enum TID : uint8_t
  {
    TID_POLY_TRAJ = 2,
    TID_FORMATION_COMMAND = 3,
  };

  JfiBridgeNode();
  ~JfiBridgeNode() = default;

private:
  /**
   * @brief Serialize a ROS2 message to CDR format
   */
  template<typename T>
  std::vector<uint8_t> serializeMessage(const T& msg);

  /**
   * @brief Deserialize a ROS2 message from CDR format
   */
  template<typename T>
  T deserializeMessage(const std::vector<uint8_t>& data);

  /**
   * @brief Callback for PolyTraj messages to send over serial
   */
  void polyTrajToSerialCallback(const path_manager::msg::PolyTraj::SharedPtr msg);

  /**
   * @brief Callback for FormationCommand messages to send over serial
   */
  void formationCommandToSerialCallback(const path_manager::msg::FormationCommand::SharedPtr msg);

  /**
   * @brief Callback for SwarmComm messages received from serial
   */
  void swarmCommFromSerialCallback(const jfi_comm::msg::SwarmComm::SharedPtr msg);

  /* ---------- Members ---------------------------------------------------- */
  uint8_t system_id_;
  int32_t last_sent_formation_cmd_sequence_;

  /* ROS Publishers -------------------------------------------------------- */
  // Outgoing to jfi_comm (ROS -> Serial)
  rclcpp::Publisher<jfi_comm::msg::SwarmComm>::SharedPtr pub_to_jfi_;

  // Incoming from serial (Serial -> ROS)
  rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr pub_poly_traj_;
  rclcpp::Publisher<path_manager::msg::FormationCommand>::SharedPtr pub_formation_cmd_;

  /* ROS Subscribers ------------------------------------------------------- */
  // From path_manager (ROS -> Serial)
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr sub_poly_traj_;
  rclcpp::Subscription<path_manager::msg::FormationCommand>::SharedPtr sub_formation_cmd_;

  // From jfi_comm (Serial -> ROS)
  rclcpp::Subscription<jfi_comm::msg::SwarmComm>::SharedPtr sub_from_jfi_;

  /* Serialization helpers ------------------------------------------------- */
  rclcpp::Serialization<path_manager::msg::PolyTraj> poly_traj_serializer_;
  rclcpp::Serialization<path_manager::msg::FormationCommand> formation_cmd_serializer_;
};

#endif  // JFI_BRIDGE_NODE_HPP
