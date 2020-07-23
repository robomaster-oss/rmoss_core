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
#ifndef RM_CAM_USB_CAM_DEV_H
#define RM_CAM_USB_CAM_DEV_H
#include "rm_cam/cam_dev_interface.h"
#include <opencv2/opencv.hpp>

namespace rm_cam {
    //the usb camera (UVC) device, based on opencv
    class UsbCamDev : public CamDevInterface{
    public:
        UsbCamDev(const std::string dev_path = "/dev/video0",const std::string conf_path = "");
        ~UsbCamDev();
    public:
        bool open();
        bool isOpened();
        int capImg(cv::Mat &img);
        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);
        bool setParameters(std::string config_path);

    private:
        bool setExposure(int value);
    private:
        std::string dev_path_;
        std::string conf_path_;
        cv::VideoCapture cap_;
        //parameters
        int cam_height_;
        int cam_width_;
        int cam_fps_;
        //flag
        bool is_open_;
    };
}

#endif //RM_CAM_USB_CAM_DEV_H



