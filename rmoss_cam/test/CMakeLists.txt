find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_cam_server test_cam_server.cpp)
target_link_libraries(test_cam_server ${PROJECT_NAME})

ament_add_gtest(test_cam_client test_cam_client.cpp)
target_link_libraries(test_cam_client ${PROJECT_NAME})

