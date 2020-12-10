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

#include "rm_cam/virtual_cam_dev.hpp"
#include "rm_common/log.hpp"
#include <thread>

using namespace cv;
using namespace std;
using namespace rm_cam;


void VirtualCamDev::setImageSource(const std::string image_path) {
    image_path_ = image_path;
    current_mode_ = CameraMode::image;
}

void VirtualCamDev::setVideoSource(const std::string video_path) {
    video_path_ = video_path;
    current_mode_ = CameraMode::video;
}

bool VirtualCamDev::open() {
    if (current_mode_ == CameraMode::image) {
        img_ = imread(image_path_);
        if (!img_.empty()) {
            cam_width_ = img_.cols;
            cam_height_ = img_.rows;
            is_open_ = true;
            return true;
        }else{
            RM_LOG_ERROR<<"[VirtualCamDev]:can't open image <"<<image_path_<<">."<<std::endl;
            return false;
        }
    } else if (current_mode_ == CameraMode::video) {
        if (cap_.open(video_path_)) {
            cam_height_ = cap_.get(CAP_PROP_FRAME_HEIGHT);
            cam_width_ = cap_.get(CAP_PROP_FRAME_WIDTH);
            total_frames_ = cap_.get(CAP_PROP_FRAME_COUNT);
            cam_fps_ = cap_.get(CAP_PROP_FPS);
            is_open_ = true;
            return true;
        }else{
            RM_LOG_ERROR<<"[VirtualCamDev]:can't open video <"<<video_path_<<">."<<std::endl;
            return false; 
        }
    }
    return false;
}

bool VirtualCamDev::isOpened() { return is_open_; }

bool VirtualCamDev::capImg(cv::Mat &img) {
    if (is_open_) {
        if (current_mode_ == CameraMode::image) {
            img = img_.clone();
            return true;
        } else if (current_mode_ == CameraMode::video) {
            if (cap_.read(img)) {
                current_frame++;
                if (current_frame > total_frames_ - 2) {
                    current_frame = 0;
                    cap_.set(CAP_PROP_POS_FRAMES, 0);
                }
                return true;
            }else{
                return false;
            }
        }else{
            //if CameraMode::null
            RM_LOG_ERROR<<"[VirtualCamDev]:CameraMode is null."<<std::endl;
            return false;
        }
        return false;
    } else {
        RM_LOG_ERROR<<"[VirtualCamDev]:virtual camera is not opened."<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        return false;
    }
}

bool VirtualCamDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case CamParameter::fps:
            cam_fps_ = value;
            return true;
        default:
            return false;
    }
}

bool VirtualCamDev::getParameter(CamParameter parameter, int &value) {
    switch (parameter) {
        case CamParameter::resolution_width:
            value = cam_width_;
            return true;
        case CamParameter::resolution_height:
            value = cam_height_;
            return true;
        case CamParameter::fps:
            value = cam_fps_;
            return true;
        default:
            return false;
    }
}
