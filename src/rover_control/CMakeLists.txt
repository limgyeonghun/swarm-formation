cmake_minimum_required(VERSION 3.5)
project(rover_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)

# Optional build - only if px4_msgs is available
find_package(px4_msgs QUIET)

if(px4_msgs_FOUND)
  message(STATUS "px4_msgs found - building rover_control")

  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(path_manager REQUIRED)
  find_package(geometry_msgs REQUIRED)

  include_directories(include)

  add_executable(rover_control_node src/rover_control.cpp)

  ament_target_dependencies(rover_control_node
    rclcpp
    px4_msgs
    nav_msgs
    path_manager
    geometry_msgs
  )

  install(TARGETS
    rover_control_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "px4_msgs not found - skipping rover_control build (not needed for simulation)")
endif()

ament_package()
