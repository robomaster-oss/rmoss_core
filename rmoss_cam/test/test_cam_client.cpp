// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "dummy_cam.hpp"
#include "rmoss_cam/cam_server.hpp"
#include "rmoss_cam/cam_client.hpp"

using namespace std::chrono_literals;

TEST(CamClient, callback)
{
  rclcpp::init(0, nullptr);
  auto cam_dev = std::make_shared<DummyCam>();
  auto node_options = rclcpp::NodeOptions();
  node_options.append_parameter_override("autostart", true);
  node_options.append_parameter_override("camera_name", "test_camera");
  auto node = std::make_shared<rclcpp::Node>("test_cam_server", node_options);
  auto cam_server = std::make_shared<rmoss_cam::CamServer>(node, cam_dev);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node2 = std::make_shared<rclcpp::Node>("test_cam_client");
  auto cam_client = std::make_shared<rmoss_cam::CamClient>(node2);
  executor->add_node(node);
  executor->add_node(node2);
  int num = 0;
  cam_client->set_camera_name("test_camera");
  cam_client->connect(
    [&](const cv::Mat & /*img*/, const rclcpp::Time & /*stamp*/) {
      num++;
    });
  for (int i = 0; i < 10; i++) {
    executor->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(num > 1, true);
  cam_client->disconnect();
  rclcpp::shutdown();
}

TEST(CamClient, intra_comms)
{
  rclcpp::init(0, nullptr);
  // create camera server manager
  auto node = std::make_shared<rclcpp::Node>("test_cam_server_manager");
  auto cam_server_manager = std::make_shared<rmoss_cam::CamServerManager>(
    node->get_node_logging_interface());
  // create camera server
  auto node_options = rclcpp::NodeOptions();
  node_options.append_parameter_override("autostart", true);
  node_options.append_parameter_override("camera_name", "test_camera");
  auto node1 = std::make_shared<rclcpp::Node>("test_cam_server", node_options);
  auto cam_dev = std::make_shared<DummyCam>();
  auto cam_server = std::make_shared<rmoss_cam::CamServer>(node1, cam_dev);
  cam_server_manager->add_cam_server(cam_server);
  // create camera client
  auto node2 = std::make_shared<rclcpp::Node>("test_cam_client");
  auto cam_client = std::make_shared<rmoss_cam::CamClient>(node2);
  cam_client->set_cam_server_manager(cam_server_manager);
  // spin
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node1);
  executor->add_node(node2);
  int num = 0;
  cam_client->set_camera_name("test_camera");
  cam_client->connect(
    [&](const cv::Mat & /*img*/, const rclcpp::Time & /*stamp*/) {
      num++;
    });
  for (int i = 0; i < 10; i++) {
    executor->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(num > 1, true);
  rclcpp::shutdown();
}
