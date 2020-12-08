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
#ifndef RM_COMMON_DEBUG_HPP
#define RM_COMMON_DEBUG_HPP

#include <opencv2/opencv.hpp>

#define DEBUG_TOOL(text)            \
    std::cout<<"DEBUG_TOOL() is deprecated,please use RM_DEBUG()."<<std::endl;\
    if (rm_common::isDebug()) \
        text

#define RM_DEBUG(text)             \
    if (rm_common::isDebug()) \
        text

namespace rm_common {
    //const definition of color
    const cv::Scalar blue = cv::Scalar(255, 0, 0);
    const cv::Scalar green = cv::Scalar(0, 255, 0);
    const cv::Scalar red = cv::Scalar(0, 0, 255);
    // debug config
    bool isDebug();
    void setDebug(bool is_debug);
    // draw function
    void drawRotatedRect(cv::Mat& img, cv::RotatedRect r, cv::Scalar color=green);
    void draw4Point4f(cv::Mat& img, cv::Point2f* point2fs, cv::Scalar color=green);
    void drawConvexHull(cv::Mat& img, std::vector<cv::Point2f> points, cv::Scalar color=green);
} // namespace rm_common

#endif // RM_COMMON_DEBUG_HPP
