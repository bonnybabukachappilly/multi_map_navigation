#include "multi_map_nav/navigate_to_map_action_server.hpp"

NavigateToMapActionServer::NavigateToMapActionServer()
    : Node("navigate_to_map_action_server"),
      current_map_name_("map1") // Simulated current map
{
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<NavigateToMap>(
        this,
        "navigate_to_map",
        std::bind(&NavigateToMapActionServer::handle_goal, this, _1, _2),
        std::bind(&NavigateToMapActionServer::handle_cancel, this, _1),
        std::bind(&NavigateToMapActionServer::execute, this, _1));

    RCLCPP_INFO(this->get_logger(), "NavigateToMap action server ready");
}

rclcpp_action::GoalResponse NavigateToMapActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToMap::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal for map '%s'", goal->target_map.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToMapActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

bool NavigateToMapActionServer::is_same_map(const std::string &target_map)
{
    return target_map == current_map_name_;
}

void NavigateToMapActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<NavigateToMap::Result>();

    std::string target_map = goal->target_map;
    geometry_msgs::msg::PoseStamped target_pose = goal->goal_pose;
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    if (current_map_name_ == target_map)
    {
        RCLCPP_INFO(this->get_logger(), "Same map (%s), navigating directly.", target_map.c_str());
        send_goal_to_nav2(target_pose); // To be implemented
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Different map. Switching from %s to %s.", current_map_name_.c_str(), target_map.c_str());

        // Step 1: Navigate to wormhole in current map
        auto wormhole_pose = get_wormhole_pose(current_map_name_, target_map);
        send_goal_to_nav2(wormhole_pose);

        // Step 2: Switch map
        switch_map(target_map); // You implement this

        // Step 3: Navigate to goal in target map
        send_goal_to_nav2(target_pose);

        // Step 4: Update current map
        current_map_name_ = target_map;
    }

    result->success = true;
    result->message = "Navigation complete.";
    goal_handle->succeed(result);
}

geometry_msgs::msg::PoseStamped NavigateToMapActionServer::get_wormhole_pose(
    const std::string &from_map, const std::string &to_map)
{
    geometry_msgs::msg::PoseStamped pose;
    sqlite3 *db;
    sqlite3_stmt *stmt;
    int rc;

    std::string db_path = "wormholes.db"; // or use full path
    rc = sqlite3_open(db_path.c_str(), &db);
    if (rc)
    {
        RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return pose;
    }

    std::string sql = R"(SELECT pose_x, pose_y, pose_theta FROM wormholes
                        WHERE from_map = ? AND to_map = ? LIMIT 1)";
    rc = sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement");
        sqlite3_close(db);
        return pose;
    }

    sqlite3_bind_text(stmt, 1, from_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, to_map.c_str(), -1, SQLITE_STATIC);

    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
        pose.pose.position.x = sqlite3_column_double(stmt, 0);
        pose.pose.position.y = sqlite3_column_double(stmt, 1);
        double theta = sqlite3_column_double(stmt, 2);
        pose.pose.orientation.z = sin(theta / 2.0);
        pose.pose.orientation.w = cos(theta / 2.0);
        RCLCPP_INFO(this->get_logger(), "Loaded wormhole pose from %s to %s", from_map.c_str(), to_map.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No wormhole found from %s to %s", from_map.c_str(), to_map.c_str());
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return pose;
}

bool NavigateToMapActionServer::send_goal_to_nav2(const geometry_msgs::msg::PoseStamped &pose)
{
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
        return false;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = pose;

    auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &wrapped_result)
    {
        // wrapped_result.code is an enum, cast it to int for printf‑style
        RCLCPP_INFO(
            rclcpp::get_logger("navigate_to_map_action_server"),
            "Navigation finished with status code: %d",
            static_cast<int>(wrapped_result.code));
    };

    auto goal_handle_future = nav2_client_->async_send_goal(goal_msg, goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2.");
        return false;
    }

    auto result_future = nav2_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result.");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded.");
        return true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "Navigation failed with code %d",
                    static_cast<int>(result.code));
        return false;
    }
}

void NavigateToMapActionServer::switch_map(const std::string &target_map)
{
    RCLCPP_INFO(this->get_logger(), "Switching to map: %s", target_map.c_str());

    // One simple way: use system call to stop/start launch files (you can make this safer later)
    std::string command = "ros2 launch multi_map_nav load_" + target_map + ".launch.py &";
    RCLCPP_INFO(this->get_logger(), "Switching map via command: %s", command.c_str());
    system(command.c_str());

    rclcpp::sleep_for(std::chrono::seconds(5));
}