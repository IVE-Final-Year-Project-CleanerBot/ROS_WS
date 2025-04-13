#ifndef BEHAVIOR_TREE_NODES_HPP
#define BEHAVIOR_TREE_NODES_HPP

#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

// 检测瓶子节点
class CheckForBottles : public BT::ConditionNode {
public:
  CheckForBottles(const std::string& name, const BT::NodeConfiguration& config)
    : ConditionNode(name, config) {
    node_ = rclcpp::Node::make_shared("check_for_bottles_node");
    client_ = node_->create_client<std_srvs::srv::SetBool>("/detection_status");
  }

  static BT::PortsList providedPorts() {
    return {BT::OutputPort<bool>("detected")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

// 控制机械臂节点
class ControlArm : public BT::SyncActionNode {
public:
  ControlArm(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {
    node_ = rclcpp::Node::make_shared("control_arm_node");
    publisher_ = node_->create_publisher<std_msgs::msg::String>("/arm_command", 10);
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif // BEHAVIOR_TREE_NODES_HPP