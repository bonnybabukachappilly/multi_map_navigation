#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "multi_map_nav/action/navigate_to_map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>


class NavigateToMapActionServer : public rclcpp::Node
{
public:
    using NavigateToMap = multi_map_nav::action::NavigateToMap;
    using GoalHandleNavigateToMap = rclcpp_action::ServerGoalHandle<NavigateToMap>;

    NavigateToMapActionServer();
    void init();

private:
    rclcpp_action::Server<NavigateToMap>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleNavigateToMap> active_goal_; // <-- ADD THIS if not present
    std::string current_map_;                              // <-- For tracking the active map

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToMap::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);
    void execute(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

    // 🔧 ADD THESE DECLARATIONS BELOW
    bool is_on_target_map(const std::string &target_map);
    bool go_to_wormhole_and_switch_map(const std::string &target_map);
    bool send_goal_to_nav2(const geometry_msgs::msg::PoseStamped &pose);
    void send_feedback(const std::string &status);
};