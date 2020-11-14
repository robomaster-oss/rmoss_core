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

#ifndef RM_TOOL_IMAGE_TOOL_HPP
#define RM_TOOL_IMAGE_TOOL_HPP

#include "opencv2/opencv.hpp"

namespace rm_tool {

class ImageTool{
public:
    static float calc2PointDistance(cv::Point2f point1,cv::Point2f point2);
    static float calc2PointDistance(cv::Point3f point1,cv::Point3f point2);
    static float calc2PointAngle(cv::Point2f point1,cv::Point2f point2);
    static float calcTriangleInnerAngle(cv::Point2f vertexPoint,cv::Point2f point1,cv::Point2f point2);    
    static bool calcCircle3Point(cv::Point2f p1,cv::Point2f p2,cv::Point2f p3,cv::Point2f &point,float &r);
};

} // namespace rm_tool

#endif //RM_TOOL_IMAGE_TOOL_HPP

