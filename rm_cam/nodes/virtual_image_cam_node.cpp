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

#include "rm_cam/camera_task.hpp"
#include "rm_cam/virtual_cam_dev.hpp"

using namespace rm_cam;

int main(int argc, char* argv[]) {
    // creat ros2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("virtual_image_cam");
    // declare parameter
    node->declare_parameter("image_path");
    rclcpp::Parameter str_param = node->get_parameter("image_path");
    std::string image_path = str_param.as_string();
    int fps = node->declare_parameter("cam_fps", 10);
    // create device
    auto cam_dev = std::make_shared<VirtualCamDev>();
    cam_dev->setImageSource(image_path);
    cam_dev->setParameter(CamParameter::fps, fps);
    cam_dev->open();
    // create a camera Task
    auto cam_task = std::make_shared<CameraTask>(node, cam_dev);
    // run node until it's exited
    rclcpp::spin(node);
    // clean up
    rclcpp::shutdown();
    return 0;
}
