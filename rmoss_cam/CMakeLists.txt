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
find_package(rmoss_util REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rmoss_cam lib
add_library(${PROJECT_NAME} SHARED
  src/cam_server.cpp
  src/cam_client.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  rmoss_interfaces
  rmoss_util
  cv_bridge
  camera_info_manager
  image_transport
  OpenCV
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# usb_cam component
add_library(usb_cam_component SHARED
  src/usb_cam/usb_cam.cpp
  src/usb_cam/usb_cam_node.cpp
)
target_link_libraries(usb_cam_component ${PROJECT_NAME})
rclcpp_components_register_node(usb_cam_component
  PLUGIN "rmoss_cam::UsbCamNode"
  EXECUTABLE usb_cam)

# virtual_cam component
add_library(virtual_cam_component SHARED
  src/virtual_cam/virtual_cam.cpp
  src/virtual_cam/virtual_cam_node.cpp
)
target_link_libraries(virtual_cam_component ${PROJECT_NAME})
rclcpp_components_register_node(virtual_cam_component
  PLUGIN "rmoss_cam::VirtualCamNode"
  EXECUTABLE virtual_cam)

# library for demo
add_library(image_task_demo_component SHARED
  src/image_task_demo_node.cpp
)
target_link_libraries(image_task_demo_component ${PROJECT_NAME})
rclcpp_components_register_node(image_task_demo_component
  PLUGIN "rmoss_cam::ImageTaskDemoNode"
  EXECUTABLE image_task_demo)

# executable
add_executable(benchmark_test src/benchmark_test_main.cpp)
target_link_libraries(benchmark_test ${PROJECT_NAME} virtual_cam_component)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME} usb_cam_component virtual_cam_component image_task_demo_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executable nodes
install(TARGETS benchmark_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install  directories
install(DIRECTORY launch resource config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# specific order: dependents before dependencies
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
