cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 包含头文件目录
include_directories(include)

# 添加可执行文件
add_executable(bt_main src/bt_main.cpp src/navigate_to_goal_node.cpp src/detect_bottle_node.cpp src/pick_up_bottle_node.cpp src/resume_navigation_node.cpp)

# 链接依赖项
ament_target_dependencies(bt_main rclcpp behaviortree_cpp_v3)

# 安装目标
install(TARGETS bt_main DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
