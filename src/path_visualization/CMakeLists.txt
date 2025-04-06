cmake_minimum_required(VERSION 3.5)
project(path_visualization)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(path_optimizer REQUIRED)
find_package(path_planner REQUIRED)  # path_planner 추가

# 라이브러리 타겟 생성
add_library(${PROJECT_NAME}_lib
  src/path_visualization.cpp
  src/node.cpp
  # dyn_a_star.cpp와 grid_map.cpp 제거
)

# 인클루드 디렉토리 설정
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 의존성 링크
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  visualization_msgs
  Eigen3
  px4_msgs
  path_optimizer
  path_planner  # path_planner 추가
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
  src/path_visualization.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  px4_msgs
  path_optimizer
  path_planner  # path_planner 추가
)

# 설치 규칙
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 내보내기 설정
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs visualization_msgs Eigen3 px4_msgs path_optimizer path_planner)  # path_planner 추가

ament_package()