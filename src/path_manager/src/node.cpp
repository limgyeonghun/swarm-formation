#include <rclcpp/rclcpp.hpp>
#include "path_manager/replan_fsm.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_manager::ReplanFSM>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}