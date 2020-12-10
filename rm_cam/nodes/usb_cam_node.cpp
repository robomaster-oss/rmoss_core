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
#include "rm_cam/usb_cam_dev.hpp"
#include "rm_common/log.hpp"

using namespace rm_cam;

int main(int argc, char* argv[]) {
    // creat ros2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("usb_cam");
    rm_common::rclcpp_log::setLogName(node->get_name());
    // declare parameter
    std::string dev_name =
        node->declare_parameter("usb_cam_path", "/dev/video0");
    int wigth = node->declare_parameter("cam_wigth", 640);
    int height = node->declare_parameter("cam_height", 480);
    int fps = node->declare_parameter("cam_fps", 10);
    // create device
    auto cam_dev = std::make_shared<UsbCamDev>(dev_name);
    cam_dev->setParameter(CamParameter::resolution_width, wigth);
    cam_dev->setParameter(CamParameter::resolution_height, height);
    cam_dev->setParameter(CamParameter::fps, fps);
    cam_dev->open();
    // create a camera node
    auto cam_task = std::make_shared<CameraTask>(node, cam_dev);
    // run node until it's exited
    rclcpp::spin(node);
    // clean up
    rclcpp::shutdown();
    return 0;
}
