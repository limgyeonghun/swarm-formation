#ifndef FORMATION_MANAGER_H
#define FORMATION_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include "path_manager/msg/formation_command.hpp"
#include "path_manager/msg/formation_target.hpp"
#include "swarm_graph/swarm_graph.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <string>

namespace path_manager
{

class FormationManager : public rclcpp::Node
{
public:
    FormationManager();

private:
    void formationCommandCallback(const path_manager::msg::FormationCommand::SharedPtr msg);
    void dronePositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int drone_id);
    void generateFormationTargets(const Eigen::Vector3d& center, const std::string& formation_type, double scale);
    std::vector<Eigen::Vector3d> generateFormationPattern(const std::string& formation_type, int num_drones, double scale);
    void publishFormationTargets(const std::vector<Eigen::Vector3d>& targets);
    void updateSwarmGraph();

    // ROS2 publishers and subscribers
    rclcpp::Subscription<path_manager::msg::FormationCommand>::SharedPtr formation_cmd_sub_;
    rclcpp::Publisher<path_manager::msg::FormationTarget>::SharedPtr formation_target_pub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_pose_subs_;

    // Formation parameters
    int num_drones_;
    std::string current_formation_type_;
    double current_formation_scale_;
    Eigen::Vector3d current_formation_center_;

    // SwarmGraph for formation optimization
    SwarmGraph::Ptr swarm_graph_;
    std::vector<Eigen::Vector3d> current_positions_;
    std::vector<bool> position_received_;
    bool has_formation_command_;
    bool all_positions_received_;

    int formation_command_count_;
};

} // namespace path_manager

#endif // FORMATION_MANAGER_H

