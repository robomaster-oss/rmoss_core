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
#include "rm_cam/usb_cam_dev.h"

using namespace rm_cam;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //create device
  auto cam_dev = std::make_shared<UsbCamDev>("/dev/video0");
  cam_dev->open();
  // create a node
  auto node = std::make_shared<CameraNode>("top_camera");
  node->init(cam_dev.get(),"top_camera/image_raw");
  // run node until it's exited
  rclcpp::spin(node);

  //clean up 
  rclcpp::shutdown();
  return 0;
}

