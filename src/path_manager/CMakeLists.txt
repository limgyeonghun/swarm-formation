cmake_minimum_required(VERSION 3.5)
project(path_manager)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Set optimization flags  
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(path_optimizer REQUIRED)
find_package(path_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(swarm_graph REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  "msg/PositionCommand.msg"
  "msg/FormationTarget.msg"
  "msg/FormationCommand.msg"
  DEPENDENCIES std_msgs builtin_interfaces nav_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_lib
  src/path_manager.cpp
  src/replan_fsm.cpp
  src/uniform_bspline.cpp
  src/polynomial_traj.cpp
  src/hungarian_algorithm.cpp
  ../common/log_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  px4_msgs
  nav_msgs
  Eigen3
  path_optimizer
  path_planner
  geometry_msgs
  swarm_graph
  rosidl_default_runtime
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  px4_msgs
  nav_msgs
  path_optimizer
  path_planner
  geometry_msgs
  rosidl_default_runtime
)

add_executable(traj_server
  src/traj_server.cpp
)
target_link_libraries(traj_server
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  px4_msgs
  nav_msgs
  path_optimizer
  path_planner
  geometry_msgs
  rosidl_default_runtime
)


add_executable(formation_commander
  src/formation_commander.cpp
)
ament_target_dependencies(formation_commander
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
  path_optimizer
  rosidl_default_runtime
)
target_include_directories(formation_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${EIGEN3_INCLUDE_DIRS}
)

rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(traj_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


rosidl_target_interfaces(formation_commander
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS traj_server
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS formation_commander
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs px4_msgs nav_msgs Eigen3 path_optimizer geometry_msgs path_planner swarm_graph rosidl_default_generators)

ament_package()