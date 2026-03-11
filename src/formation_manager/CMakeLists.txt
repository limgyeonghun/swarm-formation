cmake_minimum_required(VERSION 3.8)
project(formation_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(formation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(swarm_graph REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Formation manager node
add_executable(formation_manager_node
  src/formation_manager_node.cpp
  src/formation_utils.cpp
  src/hungarian_algorithm.cpp
)

ament_target_dependencies(formation_manager_node
  rclcpp
  formation_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  swarm_graph
  ament_index_cpp
  Eigen3
)

target_link_libraries(formation_manager_node
  yaml-cpp
)

target_include_directories(formation_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS formation_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
