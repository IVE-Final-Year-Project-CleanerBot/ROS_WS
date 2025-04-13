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
  factory.registerNodeType<NavigateToPose>("NavigateToPose"); 

  // 加载行为树 XML 文件
  std::string tree_file = ament_index_cpp::get_package_share_directory("behavior_tree") + "/config/recycle_bt.xml";
  auto tree = factory.createTreeFromFile(tree_file);

  // 执行行为树
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}