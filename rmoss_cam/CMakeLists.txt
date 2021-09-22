cmake_minimum_required(VERSION 3.5)
project(rmoss_cam)

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
find_package(sensor_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rmoss_cam lib
add_library(${PROJECT_NAME} SHARED
  src/cam_driver/usb_cam.cpp
  src/cam_driver/virtual_cam.cpp
  src/cam_server.cpp
  src/cam_client.cpp
  src/usb_cam_node.cpp
  src/virtual_cam_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  rmoss_interfaces
  cv_bridge
  OpenCV
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Create executable nodes
add_executable(usb_cam src/usb_cam_main.cpp)
target_link_libraries(usb_cam ${PROJECT_NAME})

add_executable(virtual_cam src/virtual_cam_main.cpp)
target_link_libraries(virtual_cam ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_cam::UsbCamNode")
rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_cam::VirtualCamNode")

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
install(TARGETS usb_cam virtual_cam
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch res directories
install(DIRECTORY launch resource config
  DESTINATION share/${PROJECT_NAME}
)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# specific order: dependents before dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()