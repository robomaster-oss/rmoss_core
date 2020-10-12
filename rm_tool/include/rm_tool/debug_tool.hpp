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
#ifndef RM_TOOL_DEBUG_TOOL_HPP
#define RM_TOOL_DEBUG_TOOL_HPP

#include <opencv2/opencv.hpp>

#define DEBUG_TOOL(text) \
    if (rm_tool::DebugTool::is_debug) text
      

namespace rm_tool {

//调试工具类
class DebugTool{
public:
    static bool is_debug;
    //
    static cv::Scalar getColor(int type);
    static void drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type=0);
    static void draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type=0);
    static void drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type=0);
    

};

}

#endif //RM_TOOL_DEBUG_TOOL_HPP

