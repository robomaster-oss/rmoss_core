cmake_minimum_required(VERSION 3.5)
project(rmoss_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
add_subdirectory(common)
add_subdirectory(math)

add_library(${PROJECT_NAME}::common ALIAS common)
add_library(${PROJECT_NAME}::math ALIAS math)

# Install include directories
install(DIRECTORY common/include/ math/include/
  DESTINATION include
)

# Install libraries
install(TARGETS common math
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export rmoss_util lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(image_transport)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(OpenCV)
ament_export_dependencies(rmoss_interfaces)
ament_export_dependencies(rcpputils)
ament_export_dependencies(ament_index_cpp)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(Ceres)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(common/test)
  add_subdirectory(math/test)
endif()

ament_package()
