find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_cam_server test_cam_server.cpp)
target_link_libraries(test_cam_server ${PROJECT_NAME})