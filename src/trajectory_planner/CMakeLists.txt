cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(formation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(path_optimizer REQUIRED)
find_package(path_planner REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Note: path_manager msg definitions come from path_manager package
# We only use the messages, not the formation logic
find_package(path_manager REQUIRED)

# Trajectory planner library
add_library(${PROJECT_NAME}_lib
  src/path_manager.cpp
  src/polynomial_traj.cpp
  src/uniform_bspline.cpp
  ../common/log_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  nav_msgs
  path_optimizer
  path_planner
  Eigen3
  path_manager
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common>
  $<INSTALL_INTERFACE:include>
)

# Trajectory planner node
add_executable(trajectory_planner_node
  src/trajectory_planner_node.cpp
)

ament_target_dependencies(trajectory_planner_node
  rclcpp
  formation_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
  visualization_msgs
  path_optimizer
  path_planner
  Eigen3
  path_manager
)

target_link_libraries(trajectory_planner_node
  ${PROJECT_NAME}_lib
)

target_include_directories(trajectory_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
