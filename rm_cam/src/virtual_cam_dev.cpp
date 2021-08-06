// Copyright 2020 RoboMaster-OSS
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

#include "rm_cam/virtual_cam_dev.hpp"
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
            std::cout << "[VirtualCamDev]:can't open image <"<<image_path_<<">."<<std::endl;
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
            std::cout << "[VirtualCamDev]:can't open video <"<<video_path_<<">."<<std::endl;
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
            std::cout <<"[VirtualCamDev]:CameraMode is null."<<std::endl;
            return false;
        }
        return false;
    } else {
        std::cout <<"[VirtualCamDev]:virtual camera is not opened."<<std::endl;
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
