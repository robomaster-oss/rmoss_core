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
