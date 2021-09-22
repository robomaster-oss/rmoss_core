find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_fixed_packet test_fixed_packet.cpp)
target_link_libraries(test_fixed_packet ${PROJECT_NAME})

ament_add_gtest(test_fixed_packet_tool test_fixed_packet_tool.cpp)
target_link_libraries(test_fixed_packet_tool ${PROJECT_NAME})

ament_add_gtest(test_udp_transporter test_udp_transporter.cpp)
target_link_libraries(test_udp_transporter ${PROJECT_NAME})