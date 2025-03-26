cmake_minimum_required(VERSION 3.5)
project(nav2_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(nav2_node src/nav2_node.cpp)
ament_target_dependencies(nav2_node rclcpp nav2_msgs)

install(TARGETS nav2_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
