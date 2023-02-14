cmake_minimum_required(VERSION 3.5)
project(rmoss_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include
include_directories(include)

# Create rmoss_base lib
add_library(${PROJECT_NAME} SHARED
  src/transporter_driver/uart_transporter.cpp
  src/transporter_driver/udp_transporter.cpp
  src/simple_robot_base_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rmoss_interfaces
  geometry_msgs
)

# Create robot_base_example node
add_executable(simple_robot_base src/simple_robot_base_main.cpp)
target_link_libraries(simple_robot_base ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_base::SimpleRobotBaseNode")

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

# Install executable nodes
install(TARGETS simple_robot_base
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS
  scripts/test_control_chassis.py
  scripts/test_control_gimbal.py
  DESTINATION lib/${PROJECT_NAME})

# Export rmoss_base lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rmoss_interfaces)
ament_export_dependencies(geometry_msgs)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()