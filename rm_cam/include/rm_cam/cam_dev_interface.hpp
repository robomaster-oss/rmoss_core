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
#ifndef RM_CAM_CAM_DEV_INTERFACE_HPP
#define RM_CAM_CAM_DEV_INTERFACE_HPP

#include <opencv2/opencv.hpp>

namespace rm_cam {
enum class CamParameter {
    resolution_width,
    resolution_height,
    auto_exposure,
    exposure,
    brightness,
    auto_white_balance,
    white_balance,
    gain,
    gamma,
    contrast,
    saturation,
    hue,
    fps
};

// common interface for camera device (usb cam,virtual cam,etc.)
class CamDevInterface {
   public:
    // major interface (required)
    virtual bool isOpened() = 0;
    virtual bool capImg(cv::Mat& img) = 0;
    // set and get parameter interface (optional)
    virtual bool setParameter(CamParameter /*parameter*/, int /*value*/) {
        return false;
    };
    virtual bool getParameter(CamParameter /*parameter*/, int& /*value*/) {
        return false;
    };
};
}  // namespace rm_cam

#endif  // RM_CAM_CAM_DEV_INTERFACE_HPP
