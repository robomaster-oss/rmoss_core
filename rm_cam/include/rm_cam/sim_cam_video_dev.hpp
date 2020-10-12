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
#ifndef RM_CAM_SIM_CAM_VIDEO_DEV_HPP
#define RM_CAM_SIM_CAM_VIDEO_DEV_HPP

#include <opencv2/opencv.hpp>
#include "rm_cam/cam_dev_interface.hpp"

namespace rm_cam {
    // simulated camera device by using viedo, based on opencv
    class SimCamVideoDev : public CamDevInterface{
    public:
        SimCamVideoDev(const std::string viedo_path);
        ~SimCamVideoDev();
    public:
        bool open();
        bool isOpened();
        int capImg(cv::Mat &img);
        
        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);

    private:
        std::string video_path_;
        cv::VideoCapture cap_;
        int total_frames_;
        int current_frame;
        //para
        float cam_fps_;
        //resolution
        int cam_width_;
        int cam_height_;
        //flag
        bool is_open_;
    };
}

#endif //RM_CAM_SIM_CAM_VIDEO_DEV_HPP



