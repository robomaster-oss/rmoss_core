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
#include "rm_task/task_show_image.hpp"

using namespace rm_task;

int main(int argc, char * argv[])
{
  //creat ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_show_image");
  // create task
  auto task = std::make_shared<TaskShowImage>(node);
  // run node until it's exited
  rclcpp::spin(node);
  //clean up 
  rclcpp::shutdown();
  return 0;
}

