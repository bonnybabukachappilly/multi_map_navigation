#include "multi_map_nav/navigate_to_map_action_server.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using NavigateToMap = multi_map_nav::action::NavigateToMap;
using GoalHandleNavigateToMap = rclcpp_action::ServerGoalHandle<NavigateToMap>;

NavigateToMapActionServer::NavigateToMapActionServer()
    : Node("navigate_to_map_action_server")
{
    RCLCPP_INFO(this->get_logger(), "NavigateToMapActionServer initialized.");
}

void NavigateToMapActionServer::init()
{
    action_server_ = rclcpp_action::create_server<NavigateToMap>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "navigate_to_map",
        std::bind(&NavigateToMapActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigateToMapActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigateToMapActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "NavigateToMapActionServer action server created.");

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        shared_from_this(), "navigate_to_pose");

    while (!nav2_client_->wait_for_action_server(1s))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for Nav2 action server...");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to Nav2 action server.");
}

rclcpp_action::GoalResponse NavigateToMapActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToMap::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal to map: %s", goal->target_map.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToMapActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToMapActionServer::handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
    std::thread{std::bind(&NavigateToMapActionServer::execute, this, goal_handle)}.detach();
}

void NavigateToMapActionServer::execute(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    if (!is_on_target_map(goal->target_map))
    {
        send_feedback("Not on target map. Going to wormhole...");
        if (!go_to_wormhole_and_switch_map(goal->target_map))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch maps.");
            auto result = std::make_shared<NavigateToMap::Result>();
            result->success = false;
            result->message = "Failed to switch maps.";
            (void)goal_handle->abort(result);
            return;
        }
    }

    send_feedback("Navigating to final goal...");
    if (!send_goal_to_nav2(goal->target_pose))
    {
        auto result = std::make_shared<NavigateToMap::Result>();
        result->success = false;
        result->message = "Failed to reach goal.";
        (void)goal_handle->abort(result);
        return;
    }

    auto result = std::make_shared<NavigateToMap::Result>();
    result->success = true;
    result->message = "Goal reached!";
    (void)goal_handle->succeed(result);
}

bool NavigateToMapActionServer::is_on_target_map(const std::string &target_map)
{
    return current_map_ == target_map;
}

bool NavigateToMapActionServer::go_to_wormhole_and_switch_map(const std::string &target_map)
{
    // TEMP STUB: We'll replace this with actual SQLite + map switch logic
    RCLCPP_INFO(this->get_logger(), "Going to wormhole...");
    std::this_thread::sleep_for(2s);

    // Simulate map switch
    current_map_ = target_map;
    RCLCPP_INFO(this->get_logger(), "Switched to map: %s", current_map_.c_str());
    return true;
}

void NavigateToMapActionServer::send_feedback(const std::string &status)
{
    auto feedback = std::make_shared<NavigateToMap::Feedback>();
    feedback->current_status = status;
    if (active_goal_)
    {
        active_goal_->publish_feedback(feedback);
    }
}

bool NavigateToMapActionServer::send_goal_to_nav2(const geometry_msgs::msg::PoseStamped &pose)
{
    if (!nav2_client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
        return false;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = pose;

    auto future_goal_handle = nav2_client_->async_send_goal(goal_msg);

    // Use temporary executor to avoid spinning this node again
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());

    if (executor.spin_until_future_complete(future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2.");
        return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2.");
        return false;
    }

    auto result_future = nav2_client_->async_get_result(goal_handle);
    if (executor.spin_until_future_complete(result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result from Nav2.");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully reached goal.");
        return true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to reach goal. Result code: %d", static_cast<int>(result.code));
        return false;
    }
}
