/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 * 
 *  file  : task_usb_cam.h
 *  brief : usb cam task
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2020-7-20
 ***************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "rm_cam/camera_task.h"
#include "rm_cam/sim_cam_video_dev.h"

using namespace rm_cam;

int main(int argc, char * argv[])
{
  //creat ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_usb_cam");
  //declare parameter
  node->declare_parameter("video_path");
  std::string video_path = node->get_parameter("video_path").as_string();
  //create device
  auto cam_dev = std::make_shared<SimCamVideoDev>(video_path);
  cam_dev->open();
  // create a camera node
  auto cam_task = std::make_shared<CameraTask>(node,cam_dev.get());
  // run node until it's exited
  rclcpp::spin(node);
  //clean up 
  rclcpp::shutdown();
  return 0;
}

