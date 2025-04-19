#ifndef NAVIGATIONS__NAVIGATION_SERVER_HPP_
#define NAVIGATIONS__NAVIGATION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sqlite3.h>
#include "navigations/action/navigation.hpp"

namespace navigations
{
    class NavigationServer : public rclcpp::Node
    {
    public:
        using NavigateToMap = navigations::action::NavigateToMap;
        using GoalHandleNavigateToMap = rclcpp_action::ServerGoalHandle<NavigateToMap>;

        explicit NavigationServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~NavigationServer() = default;

    private:
        rclcpp_action::Server<NavigateToMap>::SharedPtr action_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateToMap::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);
        void navigate_to_goal(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);
        void switch_maps(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

        // SQLite DB connection
        sqlite3 *db_;
    };
} // namespace navigations

#endif // NAVIGATIONS__NAVIGATION_SERVER_HPP_
