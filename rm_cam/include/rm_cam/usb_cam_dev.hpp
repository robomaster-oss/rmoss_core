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
#ifndef RM_CAM_USB_CAM_DEV_HPP
#define RM_CAM_USB_CAM_DEV_HPP

#include <opencv2/opencv.hpp>
#include "rm_cam/cam_dev_interface.hpp"

namespace rm_cam {
    //the usb camera (UVC) device, based on opencv
    class UsbCamDev : public CamDevInterface{
    public:
        UsbCamDev(const std::string dev_path);
        ~UsbCamDev();
    public:
        bool open();
        bool isOpened();
        bool capImg(cv::Mat &img);
        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);
    private:
        bool setExposure(int value);
    private:
        std::string dev_path_;
        cv::VideoCapture cap_;
        //parameters
        int cam_height_;
        int cam_width_;
        int cam_fps_;
        //flag
        bool is_open_;
    };
}

#endif //RM_CAM_USB_CAM_DEV_HPP



