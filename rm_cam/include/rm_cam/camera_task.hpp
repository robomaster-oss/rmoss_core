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
#ifndef RM_CAM_CAMERA_TASK_HPP
#define RM_CAM_CAMERA_TASK_HPP

#include <thread>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "rm_cam/cam_dev_interface.hpp"

namespace rm_cam {
    //a general ros node class example for camera.
    class CameraTask
    {
    public:
        CameraTask(rclcpp::Node::SharedPtr &nh,std::shared_ptr<CamDevInterface> cam_intercace);
        ~CameraTask(){};
    private:
        void capThread(); 
    private:
        rclcpp::Node::SharedPtr nh_;
        //tool
        image_transport::Publisher img_pub_;
        // camera_device interface
        std::shared_ptr<CamDevInterface> cam_intercace_;
        //data
        bool run_flag_;
        std::thread cam_thread_;
        int64_t fps_period_us_;
    };
}

#endif //RM_CAM_CAMERA_TASK_HPP



