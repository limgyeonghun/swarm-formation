cmake_minimum_required(VERSION 3.5)
project(path_planner)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Add compiler optimization flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Performance optimization flags
  add_compile_options(-O3 -march=native -mtune=native)
  # Vectorization optimization
  add_compile_options(-ffast-math -funroll-loops)
  # Link time optimization
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Add OpenMP support (INTERFACE to propagate dependencies)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found and enabled for parallel processing")
else()
  message(WARNING "OpenMP not found. Parallel processing will be disabled.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/dyn_a_star.cpp
  src/grid_map.cpp
  ../common/log_manager.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  Eigen3
  grid_map_msgs
  visualization_msgs
)

# Add OpenMP compile and link options (INTERFACE to propagate dependencies)
if(OpenMP_CXX_FOUND)
  target_compile_options(${PROJECT_NAME}_lib INTERFACE ${OpenMP_CXX_FLAGS})
  target_link_options(${PROJECT_NAME}_lib INTERFACE ${OpenMP_CXX_FLAGS})
endif()

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 grid_map_msgs visualization_msgs)

ament_package()