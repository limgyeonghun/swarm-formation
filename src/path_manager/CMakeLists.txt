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
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(path_optimizer REQUIRED)
find_package(path_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(formation_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Optional PX4 support
find_package(px4_msgs QUIET)
if(px4_msgs_FOUND)
  message(STATUS "px4_msgs found - PX4 support enabled")
  add_definitions(-DHAVE_PX4_MSGS)
  set(HAVE_PX4_MSGS TRUE)
else()
  message(STATUS "px4_msgs not found - PX4 support disabled (simulation only)")
  set(HAVE_PX4_MSGS FALSE)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  "msg/PositionCommand.msg"
  "msg/FormationTarget.msg"
  "msg/FormationCommand.msg"
  DEPENDENCIES std_msgs builtin_interfaces nav_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_lib
  src/path_manager.cpp
  src/uniform_bspline.cpp
  src/polynomial_traj.cpp
  src/replan_fsm.cpp
  ../common/log_manager.cpp
)

set(PATH_MANAGER_DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  Eigen3
  path_optimizer
  path_planner
  geometry_msgs
  formation_msgs
  rosidl_default_runtime
  visualization_msgs
)

if(HAVE_PX4_MSGS)
  list(APPEND PATH_MANAGER_DEPENDENCIES px4_msgs)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib
  ${PATH_MANAGER_DEPENDENCIES}
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
  ${PATH_MANAGER_DEPENDENCIES}
)

add_executable(traj_server
  src/traj_server.cpp
)
target_link_libraries(traj_server
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(traj_server
  ${PATH_MANAGER_DEPENDENCIES}
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


# formation_commander removed

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


# formation_commander removed

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(EXPORT_DEPS rclcpp std_msgs nav_msgs Eigen3 path_optimizer geometry_msgs path_planner formation_msgs rosidl_default_generators visualization_msgs)
if(HAVE_PX4_MSGS)
  list(APPEND EXPORT_DEPS px4_msgs)
endif()
ament_export_dependencies(${EXPORT_DEPS})

ament_package()