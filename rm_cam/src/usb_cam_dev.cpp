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

#include <thread>
#include "rm_cam/usb_cam_dev.hpp"

using namespace cv;
using namespace std;
using namespace rm_cam;

UsbCamDev::UsbCamDev(const std::string dev_path) {
    is_open_ = false;
    //
    dev_path_ = dev_path;
    //
    cam_width_ = 640;
    cam_height_ = 480;
    cam_fps_ = 20;
}

UsbCamDev::~UsbCamDev() {}

bool UsbCamDev::open() {
    //can't open if it has already opened.
    if (is_open_) {
        return false;
    }
    /***open device*****/
    if (cap_.open(dev_path_)) {
        //success to open
        cap_.set(cv::CAP_PROP_FOURCC,
                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
        is_open_ = true;
        return true;
    } else {
        //fail to open
        return false;
    }
}

bool UsbCamDev::isOpened() { return is_open_; }

bool UsbCamDev::capImg(cv::Mat &img) {
    int ret = -1;
    if (cap_.isOpened()) {
        if (cap_.read(img)) {
            ret = 0;
        } else {
            ret = -2;
        }
    } else {
        ret = -1;
    }
    //check
    if (ret != 0) {
        //reconnect if camera is not opened or image is not captured.
        
        std::cout << "[UsbCamDev]:camera " << dev_path_ << " failed to capture image,reconnecting!" << std::endl;
        //重连摄像头
        is_open_ = false;
        if (cap_.isOpened()) {
            cap_.release();
        }
        if (!open()) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(500));  // 500ms
        }
        return false;
    }else{
        //return true if image is captured.
        return true;
    }
}

bool UsbCamDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case CamParameter::resolution_width:
            cam_width_=value;
            return true;
        case CamParameter::resolution_height:
            cam_height_=value;
            return true;   
        case CamParameter::exposure:
            return setExposure(value);
        case CamParameter::fps:
            cam_fps_=value;
            return true; 
        default:
            return false;
    }
}

bool UsbCamDev::getParameter(CamParameter parameter, int& value) {
    switch (parameter) {
        case CamParameter::resolution_width:
            value=cam_width_;
            return true;
        case CamParameter::resolution_height:
            value=cam_height_;
            return true;    
        case CamParameter::exposure:
            return false;
        case CamParameter::fps:
            value = cam_fps_;
            return true;
        default:
            return false;
    }
}



bool UsbCamDev::setExposure(int /*value*/){
    //TODO:setting of exposure isn't supported in OpenCV
    return false;
}
