cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# 添加行为树节点库
add_library(behavior_tree_nodes SHARED src/behavior_tree_nodes.cpp)
ament_target_dependencies(behavior_tree_nodes
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
  ament_index_cpp
)

# 添加行为树主程序
add_executable(bt_main src/bt_main.cpp)
target_link_libraries(bt_main behavior_tree_nodes)
ament_target_dependencies(bt_main
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
  ament_index_cpp
)

# 安装目标
install(TARGETS
  behavior_tree_nodes
  bt_main
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()