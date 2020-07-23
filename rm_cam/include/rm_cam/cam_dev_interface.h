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
#ifndef RM_CAM_CAM_DEV_INTERFACE_H
#define RM_CAM_CAM_DEV_INTERFACE_H

#include <opencv2/opencv.hpp>
#define UNUSED(expr) do { (void)(expr); } while (0)

namespace rm_cam {
    enum CamParameter {ResolutionWidth,ResolutionHeight,Exposure,Brightness,
                    WhiteBalance,Gain,Gamma,Contrast,Saturation,Hue,Fps};

    //common interface for camera device (usb cam,simulated cam,etc.)
    class CamDevInterface{
    public:
        //major interface (required)
        virtual bool isOpened()=0;
        virtual int capImg(cv::Mat &img)=0;
        //set and get parameter interface (optional)
        virtual bool setParameter(CamParameter parameter,int value){
            UNUSED(parameter);
            UNUSED(value);
            return false;
        };
        virtual bool getParameter(CamParameter parameter,int& value){
            UNUSED(parameter);
            UNUSED(value);
            return false;};
    };
}

#endif //RM_CAM_CAM_DEV_INTERFACE_H



