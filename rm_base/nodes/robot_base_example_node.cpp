/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
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

