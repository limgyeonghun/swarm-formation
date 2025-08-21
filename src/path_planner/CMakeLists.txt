cmake_minimum_required(VERSION 3.5)
project(path_planner)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일러 최적화 플래그 추가
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 성능 최적화 플래그
  add_compile_options(-O3 -march=native -mtune=native)
  # 벡터화 최적화
  add_compile_options(-ffast-math -funroll-loops)
  # 링크 타임 최적화
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# OpenMP 지원 추가 (INTERFACE로 설정하여 의존성 전파)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found and enabled for parallel processing")
else()
  message(WARNING "OpenMP not found. Parallel processing will be disabled.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/dyn_a_star.cpp
  src/grid_map.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  Eigen3
)

# OpenMP 컴파일 및 링크 옵션 추가 (INTERFACE로 설정하여 의존성 전파)
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
ament_export_dependencies(rclcpp Eigen3)

ament_package()