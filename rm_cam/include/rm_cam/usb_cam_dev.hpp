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
        int capImg(cv::Mat &img);
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



