cmake_minimum_required(VERSION 3.5)
project(rm_cam)

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
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)
# Create rm_cam lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    sensor_msgs
    image_transport 
    cv_bridge
    OpenCV
)

# Create executable nodes
add_executable(usb_cam nodes/usb_cam_node.cpp)
target_link_libraries(usb_cam ${PROJECT_NAME})

add_executable(virtual_image_cam nodes/virtual_image_cam_node.cpp)
target_link_libraries(virtual_image_cam ${PROJECT_NAME})

add_executable(virtual_video_cam nodes/virtual_video_cam_node.cpp)
target_link_libraries(virtual_video_cam ${PROJECT_NAME})

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
)

# Install executable nodes
install(TARGETS usb_cam virtual_image_cam virtual_video_cam
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch res directories
install(DIRECTORY launch res
  DESTINATION share/${PROJECT_NAME}
)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs) 
ament_export_dependencies(image_transport) 
ament_export_dependencies(cv_bridge) 
ament_export_dependencies(OpenCV)   

ament_package()