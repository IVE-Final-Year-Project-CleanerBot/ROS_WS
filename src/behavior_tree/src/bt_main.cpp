#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behavior_tree/behavior_tree_nodes.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("behavior_tree_node");

  BT::BehaviorTreeFactory factory;

  // 注册自定义节点
  factory.registerNodeType<CheckForBottles>("CheckForBottles");
  factory.registerNodeType<ControlArm>("ControlArm");
  factory.registerNodeType<StopNavigation>("StopNavigation");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<ResumeNavigation>("ResumeNavigation");

  // 注册 NavigateToPose 节点，传递 ROS 2 节点
  factory.registerSimpleAction("NavigateToPose", [&node](const BT::NodeConfiguration& config) {
    return std::make_unique<NavigateToPose>("NavigateToPose", config, node);
  });

  // 注册行为树文件
  std::string tree_file = ament_index_cpp::get_package_share_directory("behavior_tree") + "/config/recycle_bt.xml";
  factory.registerBehaviorTreeFromFile(tree_file);

  // 创建行为树
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTree("MainTree", blackboard);

  // 执行行为树
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}