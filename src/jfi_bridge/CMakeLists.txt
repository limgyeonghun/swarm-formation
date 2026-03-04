cmake_minimum_required(VERSION 3.8)
project(jfi_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Optional JFI communication support
# jfi_comm is a submodule - only build if available
find_package(jfi_comm QUIET)

if(NOT jfi_comm_FOUND)
  message(STATUS "jfi_comm not found - skipping jfi_bridge build")
  message(STATUS "This is normal if J-Fi submodule is not initialized")
  message(STATUS "To enable: git submodule update --init --recursive")
  ament_package()
  return()
endif()

message(STATUS "jfi_comm found - building jfi_bridge")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(path_manager REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(
  include
)

# JFi Bridge Node executable
add_executable(jfi_bridge_node
  src/jfi_bridge_node.cpp
  src/main.cpp
)

ament_target_dependencies(jfi_bridge_node
  rclcpp
  std_msgs
  path_manager
  jfi_comm
  rclcpp_components
)

# Install targets
install(TARGETS
  jfi_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
