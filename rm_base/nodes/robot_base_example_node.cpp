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
#include <rclcpp/rclcpp.hpp>
#include "rm_base/robot_base_example.hpp"
#include "rm_base/serialport_dev.hpp"

using namespace rm_base;

int main(int argc, char * argv[])
{
  //create ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_base");
  //creat communication device
  auto comm_dev=std::make_shared<rm_base::SerialPortDev>();
  comm_dev->init("/dev/ttyUSB0");
  // create RobotBase Task
  auto robot_base_example = std::make_shared<RobotBaseExample>(node,comm_dev);
  // run node until it's exited
  rclcpp::spin(node);
  //clean up 
  rclcpp::shutdown();
  return 0;
}

