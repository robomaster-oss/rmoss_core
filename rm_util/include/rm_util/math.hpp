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
    float calcInclineAngle(cv::Point2f point1, cv::Point2f point2);
    float calcInnerAngle(cv::Point2f vertexPoint, cv::Point2f point1, cv::Point2f point2);
    bool calcCircle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f& point, float& r);
}

#endif //RM_UTIL_MATH_HPP
