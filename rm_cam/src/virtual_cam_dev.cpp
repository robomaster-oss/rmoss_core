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

#include <thread>

using namespace cv;
using namespace std;
using namespace rm_cam;


void VirtualCamDev::setImageSource(const std::string image_path) {
    image_path_ = image_path;
    current_mode_ = IMAGE_MODE;
}

void VirtualCamDev::setVideoSource(const std::string video_path) {
    video_path_ = video_path;
    current_mode_ = VIDEO_MODE;
}

bool VirtualCamDev::open() {
    if (current_mode_ == IMAGE_MODE) {
        img_ = imread(image_path_);
        if (!img_.empty()) {
            cam_width_ = img_.cols;
            cam_height_ = img_.rows;
            is_open_ = true;
            return true;
        }
    } else if (current_mode_ == VIDEO_MODE) {
        if (cap_.open(video_path_)) {
            cam_height_ = cap_.get(CAP_PROP_FRAME_HEIGHT);
            cam_width_ = cap_.get(CAP_PROP_FRAME_WIDTH);
            total_frames_ = cap_.get(CAP_PROP_FRAME_COUNT);
            cam_fps_ = cap_.get(CAP_PROP_FPS);
            is_open_ = true;
            return true;
        }
    }
    return false;
}

bool VirtualCamDev::isOpened() { return is_open_; }

int VirtualCamDev::capImg(cv::Mat &img) {
    if (is_open_) {
        if (current_mode_ == IMAGE_MODE) {
            img = img_.clone();
            return 0;
        } else if (current_mode_ == VIDEO_MODE) {
            if (cap_.read(img)) {
                current_frame++;
                if (current_frame > total_frames_ - 2) {
                    current_frame = 0;
                    cap_.set(CAP_PROP_POS_FRAMES, 0);
                }
                return 0;
            }
        }
        return -2;
    } else {
        cout << "virtual camera error!" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return -1;
    }
}

bool VirtualCamDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case Fps:
            cam_fps_ = value;
            return true;
        default:
            return false;
    }
}

bool VirtualCamDev::getParameter(CamParameter parameter, int &value) {
    switch (parameter) {
        case ResolutionWidth:
            value = cam_width_;
            return true;
        case ResolutionHeight:
            value = cam_height_;
            return true;
        case Fps:
            value = cam_fps_;
            return true;
        default:
            return false;
    }
}
