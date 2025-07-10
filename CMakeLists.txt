cmake_minimum_required(VERSION 3.5)
project(patrol_behavior)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(patrol_node src/patrol_node.cpp)
ament_target_dependencies(patrol_node rclcpp rclcpp_action nav2_msgs geometry_msgs)
target_link_libraries(patrol_node yaml-cpp)

install(TARGETS patrol_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
