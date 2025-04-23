#include <rclcpp/rclcpp.hpp>
#include "path_manager/replan_fsm.h"
#include <thread>
#include <chrono>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_manager::ReplanFSM>();
    node->init();

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
