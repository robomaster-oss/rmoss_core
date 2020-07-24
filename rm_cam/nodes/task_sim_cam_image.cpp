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
#include "rm_cam/camera_node.h"
#include "rm_cam/sim_cam_image_dev.h"

using namespace rm_cam;

int main(int argc, char * argv[])
{
  //creat ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_sim_cam");
  //declare parameter
  node->declare_parameter("image_path");
  rclcpp::Parameter str_param = node->get_parameter("image_path");
  std::string image_name = str_param.as_string();
  int fps = node->declare_parameter("cam_fps", 10);
  //create device
  auto cam_dev = std::make_shared<SimCamImageDev>(image_name);
  cam_dev->setParameter(Fps, fps);
  cam_dev->open();
  // create a camera node
  auto cam_node = std::make_shared<CameraNode>();
  cam_node->init(node,cam_dev.get());
  // run node until it's exited
  rclcpp::spin(node);
  //clean up 
  rclcpp::shutdown();
  return 0;
}

