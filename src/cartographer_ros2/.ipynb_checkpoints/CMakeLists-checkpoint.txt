cmake_minimum_required(VERSION 3.5)
project(cartographer_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(cartographer_node src/cartographer_node.cpp)
ament_target_dependencies(cartographer_node rclcpp sensor_msgs std_srvs)

install(TARGETS cartographer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()