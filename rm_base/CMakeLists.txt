cmake_minimum_required(VERSION 3.5)
project(rm_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

#include
include_directories(include)

#create rm_base lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    rm_interfaces
    geometry_msgs
)
#create robot_base_example node
add_executable(robot_base_example nodes/robot_base_example_node.cpp)
target_link_libraries(robot_base_example ${PROJECT_NAME})

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
install(TARGETS robot_base_example
  DESTINATION lib/${PROJECT_NAME}
)
# Install executable scripts
install(PROGRAMS 
        scripts/chassis_control_test.py 
        scripts/gimbal_control_test.py 
    DESTINATION lib/${PROJECT_NAME})

#export rm_base lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rm_interfaces)
ament_export_dependencies(geometry_msgs)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()