#include <rclcpp/rclcpp.hpp>
#include "path_visualization/path_visualization.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
