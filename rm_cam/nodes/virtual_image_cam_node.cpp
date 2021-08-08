// Copyright 2020 RoboMaster-OSS
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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_cam/cam_server.hpp"
#include "rm_cam/virtual_cam.hpp"


int main(int argc, char * argv[])
{
  // creat ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("virtual_image_cam");
  // declare parameter
  node->declare_parameter("image_path");
  std::string image_path = node->get_parameter("image_path").as_string();
  // create device
  auto cam_dev = std::make_shared<rm_cam::VirtualCam>(rm_cam::VirtualCam::IMAGE_MODE, image_path);
  // create a camera Task
  auto cam_task = std::make_shared<rm_cam::CamServer>(node, cam_dev);
  // run node until it's exited
  rclcpp::spin(node);
  // clean up
  rclcpp::shutdown();
  return 0;
}
