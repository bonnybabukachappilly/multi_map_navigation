#ifndef MULTI_MAP_NAV__NAVIGATE_TO_MAP_ACTION_SERVER_HPP_
#define MULTI_MAP_NAV__NAVIGATE_TO_MAP_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "multi_map_nav/action/navigate_to_map.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigateToMapActionServer : public rclcpp::Node
{
public:
    using NavigateToMap = multi_map_nav::action::NavigateToMap;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToMap>;

    NavigateToMapActionServer();

private:
    rclcpp_action::Server<NavigateToMap>::SharedPtr action_server_;
    std::string current_map_name_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToMap::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    geometry_msgs::msg::PoseStamped get_wormhole_pose(
        const std::string &from_map, const std::string &to_map);

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

    void execute(const std::shared_ptr<GoalHandle> goal_handle);
    bool is_same_map(const std::string &target_map);
    bool send_goal_to_nav2(const geometry_msgs::msg::PoseStamped &pose);
    void switch_map(const std::string &target_map);
};

#endif // MULTI_MAP_NAV__NAVIGATE_TO_MAP_ACTION_SERVER_HPP_
