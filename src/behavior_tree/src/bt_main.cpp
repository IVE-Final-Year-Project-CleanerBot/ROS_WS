#include "behavior_tree/behavior_tree_nodes.hpp"

// 检测瓶子节点实现
BT::NodeStatus CheckForBottles::tick() {
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto result = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    bool detected = result.get()->success;
    setOutput("detected", detected);
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

// 控制机械臂节点实现
BT::NodeStatus ControlArm::tick() {
  auto action = getInput<std::string>("action");
  if (!action) {
    throw BT::RuntimeError("Missing required input [action]");
  }

  auto msg = std_msgs::msg::String();
  msg.data = action.value();
  publisher_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "Sent arm command: %s", action.value().c_str());
  return BT::NodeStatus::SUCCESS;
}