find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)

ament_add_gtest(test_cam_server test_cam_server.cpp)
target_link_libraries(test_cam_server ${PROJECT_NAME})
ament_target_dependencies(test_cam_server ament_index_cpp)

ament_add_gtest(test_cam_client test_cam_client.cpp)
target_link_libraries(test_cam_client ${PROJECT_NAME})

