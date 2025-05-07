cmake_minimum_required(VERSION 3.5)
project(rover_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(rover_control_node src/rover_control.cpp)

ament_target_dependencies(rover_control_node
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  rover_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
