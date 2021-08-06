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

#ifndef RM_UTIL_MATH_HPP
#define RM_UTIL_MATH_HPP

#include "opencv2/opencv.hpp"

namespace rm_util {
    template<typename T>
    inline T rad_to_deg(T radian){ return radian * 180 / CV_PI; }
    template<typename T>
    inline T deg_to_rad(T degree){ return degree / 180 * CV_PI; }
    float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
    float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
    bool calc_circle_from_3points(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f& point, float& r);
}

#endif //RM_UTIL_MATH_HPP
