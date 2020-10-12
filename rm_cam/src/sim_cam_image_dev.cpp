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
#include "rm_cam/sim_cam_image_dev.hpp"

using namespace rm_cam;
using namespace cv;
using namespace std;

SimCamImageDev::SimCamImageDev(const std::string image_path) { 
    is_open_ = false;
    cam_fps_ = 30;
    image_path_ = image_path;
}

SimCamImageDev::~SimCamImageDev() {}

bool SimCamImageDev::open(){
    img_ = imread(image_path_);
    if (img_.empty()) {
        cout << "image sim cam error,please check image path:" << image_path_ << endl;
        return false;
    } 
    cam_width_=img_.cols;
    cam_height_=img_.rows;
    is_open_ = true;
    return true;
}


bool SimCamImageDev::isOpened() { return is_open_; }

int SimCamImageDev::capImg(cv::Mat& img) {
    if (is_open_) {
        img = img_.clone();
        return 0;
    } else {
        cout << "image sim cam error,cap error!" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return -1;
    }
}

bool SimCamImageDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case Fps:
            cam_fps_ = value;
            return true;
        default:
            return false;
    }
}

bool SimCamImageDev::getParameter(CamParameter parameter, int& value) {
    switch (parameter) {
        case ResolutionWidth:
            value=cam_width_;
            return true;
        case ResolutionHeight:
            value=cam_height_;
            return true;     
        case Fps:
            value = cam_fps_;
            return true;     
        default:
            return false;
    }
}
