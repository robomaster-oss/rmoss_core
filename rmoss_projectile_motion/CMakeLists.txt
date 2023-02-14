cmake_minimum_required(VERSION 3.5)
project(rmoss_projectile_motion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# include
include_directories(include)

# create rmoss_projectile_motion lib
add_library(${PROJECT_NAME} SHARED
  src/iterative_projectile_tool.cpp
  src/gravity_projectile_solver.cpp
  src/gaf_projectile_solver.cpp
  src/gimbal_transform_tool.cpp
)
set(dependencies
  rclcpp
  Eigen3
  eigen3_cmake_module
  geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export rmoss_projectile_motion lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()