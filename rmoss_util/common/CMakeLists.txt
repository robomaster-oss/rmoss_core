# find package
find_package(ament_index_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# create rmoss_util::common lib
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src COMMON_SRCS)
add_library(common SHARED ${COMMON_SRCS})
ament_target_dependencies(common
  rclcpp
  ament_index_cpp
  rcpputils
  sensor_msgs
  rmoss_interfaces
  cv_bridge
  OpenCV
)
target_include_directories(common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
