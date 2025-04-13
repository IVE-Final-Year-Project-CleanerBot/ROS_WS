// behavior_tree_nodes.hpp
#include "behaviortree_cpp_v3/bt_factory.h"

class CheckForBottles : public BT::ConditionNode {
public:
  CheckForBottles(const std::string& name, const BT::NodeConfiguration& config)
    : ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<bool>("detected")};
  }

  BT::NodeStatus tick() override {
    auto detected = /* 通过ROS2服务调用获取YOLO检测状态 */;
    setOutput("detected", detected);
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ControlArm : public BT::SyncActionNode {
public:
  ControlArm(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action")};
  }

  BT::NodeStatus tick() override {
    auto action = getInput<std::string>("action");
    /* 调用机械臂服务 */
    return BT::NodeStatus::SUCCESS;
  }
};