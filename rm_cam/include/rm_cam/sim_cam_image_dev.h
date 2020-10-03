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
#ifndef RM_CAM_SIM_CAM_IMAGE_DEV_H
#define RM_CAM_SIM_CAM_IMAGE_DEV_H

#include "rm_cam/cam_dev_interface.h"
#include <opencv2/opencv.hpp>


namespace rm_cam {
    //the simulated camera device by using image, based on opencv
    class SimCamImageDev : public CamDevInterface{
    public:
        SimCamImageDev(const std::string image_path);
        ~SimCamImageDev();
    public:
        bool open();
        bool isOpened();
        int capImg(cv::Mat &img);

        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);

    private:
        std::string image_path_;
        cv::Mat img_; //img cap buffer
        //resolution
        int cam_width_;
        int cam_height_;
        //flag
        bool is_open_;
        //para
        int cam_fps_;
        
    };
}

#endif //RM_CAM_SIM_CAM_IMAGE_DEV_H



