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
#ifndef RM_CAM_VIRTUAL_CAM_DEV_HPP
#define RM_CAM_VIRTUAL_CAM_DEV_HPP

#include <opencv2/opencv.hpp>
#include "rm_cam/cam_dev_interface.hpp"


namespace rm_cam {
    //the virtual camera device by using image or video, based on opencv
    enum CameraMode {NONE_MODE, IMAGE_MODE, VIDEO_MODE};
    class VirtualCamDev : public CamDevInterface{
    public:
        VirtualCamDev(){};
        ~VirtualCamDev(){};
    public:
        void setImageSource(const std::string image_path);
        void setVideoSource(const std::string video_path);
        bool open();
        bool isOpened() override;
        int capImg(cv::Mat &img) override;

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
        CameraMode current_mode_ = NONE_MODE;
    };
}

#endif //RM_CAM_VIRTUAL_CAM_DEV_HPP



