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
#ifndef RM_UTIL_DEBUG_HPP
#define RM_UTIL_DEBUG_HPP

#include <opencv2/opencv.hpp>

#define RM_DEBUG(text)             \
    if (rm_util::get_debug()) \
        text

namespace rm_util {
    //const definition of color
    const auto blue = cv::Scalar(255, 0, 0);
    const auto green = cv::Scalar(0, 255, 0);
    const auto red = cv::Scalar(0, 0, 255);

    // debug config
    bool get_debug();
    void set_debug(bool get_debug);
    // draw function
    void draw_rotated_rect(cv::Mat& img, cv::RotatedRect r, cv::Scalar color=green);
    void draw_4points(cv::Mat& img, cv::Point2f* point2fs, cv::Scalar color=green);
    void draw_convex_hull(cv::Mat& img, std::vector<cv::Point2f> points, cv::Scalar color=green);
} // namespace rm_util

#endif // RM_UTIL_DEBUG_HPP
