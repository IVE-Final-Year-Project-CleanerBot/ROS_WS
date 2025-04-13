#include "behavior_tree/behavior_tree_nodes.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// NavigateToPose 构造函数实现
NavigateToPose::NavigateToPose(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
    // 从黑板中获取 ROS 2 节点
    auto blackboard = config.blackboard;
    if (!blackboard->get("node", node_))
    {
        throw BT::RuntimeError("Missing required ROS 2 node in blackboard");
    }
}

// NavigateToPose tick 函数实现
BT::NodeStatus NavigateToPose::tick()
{
    auto goal = getInput<std::string>("goal");
    if (!goal)
    {
        throw BT::RuntimeError("Missing required input [goal]");
    }

    RCLCPP_INFO(node_->get_logger(), "Sending navigation goal: %s", goal.value().c_str());

    // 创建动作客户端
    auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

    // 等待动作服务器可用
    if (!action_client->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available!");
        return BT::NodeStatus::FAILURE;
    }

    // 解析目标点
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    std::istringstream iss(goal.value());
    char delimiter;
    iss >> goal_msg.pose.pose.position.x >> delimiter >> goal_msg.pose.pose.position.y >> delimiter >> goal_msg.pose.pose.position.z;

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();

    // 发送目标点
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [](const auto &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation succeeded!");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Navigation failed!");
        }
    };

    auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);

    // 等待导航完成
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal!");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        return BT::NodeStatus::FAILURE;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get result!");
        return BT::NodeStatus::FAILURE;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(), "Navigation succeeded!");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Navigation failed!");
        return BT::NodeStatus::FAILURE;
    }
}

// 检测瓶子节点实现
BT::NodeStatus CheckForBottles::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("CheckForBottles"), "Checking for bottles...");
    bool detected = true;
    setOutput("detected", detected);
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// 控制机械臂节点实现
BT::NodeStatus ControlArm::tick()
{
    auto action = getInput<std::string>("action");
    if (!action)
    {
        throw BT::RuntimeError("Missing required input [action]");
    }
    RCLCPP_INFO(rclcpp::get_logger("ControlArm"), "Executing arm action: %s", action.value().c_str());
    return BT::NodeStatus::SUCCESS;
}

// 停止导航节点实现
BT::NodeStatus StopNavigation::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("StopNavigation"), "Stopping navigation...");
    return BT::NodeStatus::SUCCESS;
}

// 视觉伺服对准节点实现
BT::NodeStatus ApproachObject::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("ApproachObject"), "Approaching object...");
    return BT::NodeStatus::SUCCESS;
}

// 恢复导航节点实现
BT::NodeStatus ResumeNavigation::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("ResumeNavigation"), "Resuming navigation...");
    return BT::NodeStatus::SUCCESS;
}