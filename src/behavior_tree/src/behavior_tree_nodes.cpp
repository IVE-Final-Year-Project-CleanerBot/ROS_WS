#include "behavior_tree/behavior_tree_nodes.hpp"

// 检测瓶子节点实现
BT::NodeStatus CheckForBottles::tick() {
  RCLCPP_INFO(rclcpp::get_logger("CheckForBottles"), "Checking for bottles...");
  return BT::NodeStatus::SUCCESS;
}

// 控制机械臂节点实现
BT::NodeStatus ControlArm::tick() {
  RCLCPP_INFO(rclcpp::get_logger("ControlArm"), "Controlling arm...");
  return BT::NodeStatus::SUCCESS;
}

// 导航到目标点节点实现
BT::NodeStatus NavigateToPose::tick() {
  RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigating to pose...");
  return BT::NodeStatus::SUCCESS;
}

// 停止导航节点实现
BT::NodeStatus StopNavigation::tick() {
  RCLCPP_INFO(rclcpp::get_logger("StopNavigation"), "Stopping navigation...");
  return BT::NodeStatus::SUCCESS;
}

// 视觉伺服对准节点实现
BT::NodeStatus ApproachObject::tick() {
  RCLCPP_INFO(rclcpp::get_logger("ApproachObject"), "Approaching object...");
  return BT::NodeStatus::SUCCESS;
}

// 恢复导航节点实现
BT::NodeStatus ResumeNavigation::tick() {
  RCLCPP_INFO(rclcpp::get_logger("ResumeNavigation"), "Resuming navigation...");
  return BT::NodeStatus::SUCCESS;
}