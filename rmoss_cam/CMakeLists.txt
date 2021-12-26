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
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rmoss_cam lib
add_library(${PROJECT_NAME} SHARED
  src/cam_server.cpp
  src/cam_client.cpp
  src/intra_cam_client.cpp
  src/usb_cam/usb_cam.cpp
  src/usb_cam/usb_cam_node.cpp
  src/virtual_cam/virtual_cam.cpp
  src/virtual_cam/virtual_cam_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  rmoss_interfaces
  rmoss_util
  cv_bridge
  camera_info_manager
  OpenCV
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# register component nodes and generate executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rmoss_cam::UsbCamNode"
  EXECUTABLE usb_cam)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rmoss_cam::VirtualCamNode"
  EXECUTABLE virtual_cam)

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

# Install  directories
install(DIRECTORY launch resource config scripts
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