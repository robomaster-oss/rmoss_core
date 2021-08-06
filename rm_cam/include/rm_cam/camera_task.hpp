// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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



