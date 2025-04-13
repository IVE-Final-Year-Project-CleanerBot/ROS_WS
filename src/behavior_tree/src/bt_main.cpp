#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behavior_tree/behavior_tree_nodes.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

std::string target_pose; // 全局变量，用于存储目标点
std::mutex target_pose_mutex; // 互斥锁，防止多线程冲突

void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // 将目标点转换为字符串格式并存储
  std::lock_guard<std::mutex> lock(target_pose_mutex);
  target_pose = "x:" + std::to_string(msg->pose.position.x) +
                ",y:" + std::to_string(msg->pose.position.y) +
                ",z:" + std::to_string(msg->pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger("bt_main"), "Received goal: %s", target_pose.c_str());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("behavior_tree_node");

  // 创建订阅器监听 RViz 设置的目标点
  auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, goalPoseCallback);

  BT::BehaviorTreeFactory factory;

  // 注册自定义节点
  factory.registerNodeType<CheckForBottles>("CheckForBottles");
  factory.registerNodeType<ControlArm>("ControlArm");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<StopNavigation>("StopNavigation");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<ResumeNavigation>("ResumeNavigation");

  // 注册行为树文件
  std::string tree_file = ament_index_cpp::get_package_share_directory("behavior_tree") + "/config/recycle_bt.xml";
  factory.registerBehaviorTreeFromFile(tree_file);

  // 创建行为树并设置黑板变量
  auto blackboard = BT::Blackboard::create();
  blackboard->set("target_pose", ""); // 初始化目标点为空
  auto tree = factory.createTree("MainTree", blackboard);

  // 执行行为树
  while (rclcpp::ok()) {
    {
      // 更新黑板中的目标点
      std::lock_guard<std::mutex> lock(target_pose_mutex);
      if (!target_pose.empty()) {
        blackboard->set("target_pose", target_pose);
      }
    }

    tree.tickRoot();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 增加循环间隔，避免过快
  }

  rclcpp::shutdown();
  return 0;
}