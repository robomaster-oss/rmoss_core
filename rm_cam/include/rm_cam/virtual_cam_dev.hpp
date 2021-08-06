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
#ifndef RM_CAM_VIRTUAL_CAM_DEV_HPP
#define RM_CAM_VIRTUAL_CAM_DEV_HPP

#include <opencv2/opencv.hpp>
#include "rm_cam/cam_dev_interface.hpp"


namespace rm_cam {
    //the virtual camera device by using image or video, based on opencv
    enum class CameraMode {null, image, video};
    class VirtualCamDev : public CamDevInterface{
    public:
        VirtualCamDev(){};
        ~VirtualCamDev(){};
    public:
        void setImageSource(const std::string image_path);
        void setVideoSource(const std::string video_path);
        bool open();
        bool isOpened() override;
        bool capImg(cv::Mat &img) override;

        bool setParameter(CamParameter parameter,int value) override;
        bool getParameter(CamParameter parameter,int& value) override;

    private:
        //for image
        std::string image_path_;
        cv::Mat img_;
        //for video
        std::string video_path_;
        cv::VideoCapture cap_;
        int total_frames_;
        int current_frame;
        //camera para
        int cam_width_;
        int cam_height_;
        int cam_fps_;
        //flag
        bool is_open_=false;
        CameraMode current_mode_{CameraMode::null};
    };
}

#endif //RM_CAM_VIRTUAL_CAM_DEV_HPP



