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
    : ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::OutputPort<bool>("detected")};
  }

  BT::NodeStatus tick() override;
};

// 控制机械臂节点
class ControlArm : public BT::SyncActionNode {
public:
  ControlArm(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action")};
  }

  BT::NodeStatus tick() override;
};

// 导航到目标点节点
class NavigateToPose : public BT::SyncActionNode {
public:
  NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("goal")};
  }

  BT::NodeStatus tick() override;
};

// 停止导航节点
class StopNavigation : public BT::SyncActionNode {
public:
  StopNavigation(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {}; // 无输入或输出端口
  }

  BT::NodeStatus tick() override;
};

// 视觉伺服对准节点
class ApproachObject : public BT::SyncActionNode {
public:
  ApproachObject(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {}; // 无输入或输出端口
  }

  BT::NodeStatus tick() override;
};

// 恢复导航节点
class ResumeNavigation : public BT::SyncActionNode {
public:
  ResumeNavigation(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {}; // 无输入或输出端口
  }

  BT::NodeStatus tick() override;
};

#endif