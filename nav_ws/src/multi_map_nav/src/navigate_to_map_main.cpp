#include "multi_map_nav/navigate_to_map_action_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavigateToMapActionServer>();
    node->init();  // Call init AFTER the shared_ptr exists

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
