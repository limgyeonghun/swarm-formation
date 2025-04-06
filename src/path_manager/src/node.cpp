#include <rclcpp/rclcpp.hpp>
#include "path_manager/path_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}