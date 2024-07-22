# find package
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# create rmoss_util::math lib
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src MATH_SRCS)
add_library(math SHARED ${MATH_SRCS})
ament_target_dependencies(math
  eigen3_cmake_module
  Eigen3
  Ceres
  OpenCV
)
target_include_directories(math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
