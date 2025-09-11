#include "path_manager/formation_manager.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto formation_manager = std::make_shared<path_manager::FormationManager>();
    RCLCPP_INFO(formation_manager->get_logger(), "FormationManager node started");
    rclcpp::spin(formation_manager);

    rclcpp::shutdown();
    return 0;
}

