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
#include "rm_cam/cam_server.hpp"

using namespace std::chrono_literals;

TEST(CamServer, reopen)
{
  rclcpp::init(0, nullptr);
  auto cam_dev = std::make_shared<DummyCam>();
  auto node = std::make_shared<rclcpp::Node>("test_cam");
  auto cam_server = std::make_shared<rm_cam::CamServer>(node, cam_dev);
  auto spin_thread = std::thread([&]() {rclcpp::spin(node);});
  std::this_thread::sleep_for(500ms);
  cam_dev->set_falut();
  std::this_thread::sleep_for(1000ms);
  rclcpp::shutdown();
  spin_thread.join();
  SUCCEED();
}
