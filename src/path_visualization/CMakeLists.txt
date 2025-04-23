cmake_minimum_required(VERSION 3.8)
project(path_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(path_manager REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(path_visualization_node src/path_visualization.cpp src/node.cpp)

# Specify dependencies for the executable
ament_target_dependencies(path_visualization_node
  rclcpp
  visualization_msgs
  geometry_msgs
  path_manager
  Eigen3
)

# Install targets
install(TARGETS
  path_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()