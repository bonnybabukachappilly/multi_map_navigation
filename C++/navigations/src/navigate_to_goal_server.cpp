#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sqlite3.h"
#include "action_msgs/msg/goal_status.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "example_interfaces/action/navigate_to_goal.hpp"

class NavigateToGoalServer : public rclcpp::Node
{
public:
    NavigateToGoalServer()
        : Node("navigate_to_goal_server")
    {
        action_server_ = rclcpp_action::create_server<example_interfaces::action::NavigateToGoal>(
            this, "navigate_to_goal",
            std::bind(&NavigateToGoalServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigateToGoalServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NavigateToGoalServer::handle_accepted, this, std::placeholders::_1));

        // Initialize SQLite database
        int rc = sqlite3_open("wormhole_coordinates.db", &db_);
        if (rc)
        {
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Opened database successfully");
        }
    }

    ~NavigateToGoalServer()
    {
        sqlite3_close(db_);
    }

private:
    rclcpp_action::Server<example_interfaces::action::NavigateToGoal>::SharedPtr action_server_;
    sqlite3 *db_;

    // Handle incoming goals
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const example_interfaces::action::NavigateToGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: navigating to %s", goal->target_map_name.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellations
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::NavigateToGoal>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle when the goal is accepted
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::NavigateToGoal>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Goal accepted: navigating to %s", goal->target_map_name.c_str());

        // Logic to check if the robot is in the same map
        // If not, switch maps and navigate to the goal

        if (!navigate_to_goal(goal))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to navigate to goal");
            goal_handle->succeed();
            return;
        }

        goal_handle->succeed();
    }

    bool navigate_to_goal(std::shared_ptr<const example_interfaces::action::NavigateToGoal::Goal> goal)
    {
        // Check if the goal is within the current map
        std::string target_map_name = goal->target_map_name;

        // Check the wormhole in SQLite
        double x, y;
        if (get_wormhole_coordinates(target_map_name, x, y))
        {
            RCLCPP_INFO(this->get_logger(), "Wormhole found: navigating to wormhole at (%f, %f)", x, y);
            // Code to navigate to the wormhole, switch maps, and then continue to the goal

            // Simulate map switch and navigation
            return true;
        }

        return false;
    }

    bool get_wormhole_coordinates(const std::string &map_name, double &x, double &y)
    {
        std::string query = "SELECT x, y FROM wormholes WHERE map_name = ?";
        sqlite3_stmt *stmt;
        sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, 0);
        sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);

        int rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW)
        {
            x = sqlite3_column_double(stmt, 0);
            y = sqlite3_column_double(stmt, 1);
            sqlite3_finalize(stmt);
            return true;
        }

        sqlite3_finalize(stmt);
        return false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToGoalServer>());
    rclcpp::shutdown();
    return 0;
}
