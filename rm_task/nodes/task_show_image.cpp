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
#include "rm_task/task_show_image_node.h"

using namespace rm_task;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create a node
  auto node = std::make_shared<TaskShowImageNode>("show_image_node","test_path");

  // run node until it's exited
  rclcpp::spin(node);

  //clean up 
  rclcpp::shutdown();
  return 0;
}

