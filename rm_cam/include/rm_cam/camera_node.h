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
#ifndef RM_CAM_CAMERA_NODE_H
#define RM_CAM_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>

#include "rm_cam/cam_dev_interface.h"

namespace rm_cam {
    //a general ros node class example for camera.
   class CameraNode : public rclcpp::Node
   {
   public:
     CameraNode(std::string node_name);
     ~CameraNode();
   public:
     int init(CamDevInterface *cam_intercace,std::string topic_name="camera/image_raw");
   private:
     void capThread(); 
    private:
        //tool
        image_transport::Publisher img_pub_;
        //ros::ServiceServer srv_start_, srv_stop_;
        CamDevInterface *cam_intercace_;   // camera_device interface
        //data
        bool run_flag_;
        std::thread cam_thread_;
        int64_t fps_period_us_;
  };
}

#endif //RM_CAM_CAMERA_NODE_H



