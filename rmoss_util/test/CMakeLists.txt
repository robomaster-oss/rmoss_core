find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_url_resolve test_url_resolve.cpp)
target_link_libraries(test_url_resolve ${PROJECT_NAME})