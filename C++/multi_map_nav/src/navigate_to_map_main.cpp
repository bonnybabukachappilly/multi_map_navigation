#include "multi_map_nav/navigate_to_map_action_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToMapActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
