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

#include "rm_util/debug.hpp"
using namespace std;
using namespace cv;

namespace rm_util {

bool g_is_debug = false;

bool isDebug()
{
    return g_is_debug;
}
void setDebug(bool is_debug)
{
    g_is_debug = is_debug;
}
//调试部分

//绘制旋转矩形
void drawRotatedRect(cv::Mat& img, cv::RotatedRect r, Scalar color)
{

    Point2f rect_points[4];
    r.points(rect_points);
    for (int j = 0; j < 4; j++)
        line(img, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8);
}
//绘制四边形
void draw4Point4f(cv::Mat& img, cv::Point2f* point2fs, Scalar color)
{
    for (int j = 0; j < 4; j++)
        line(img, point2fs[j], point2fs[(j + 1) % 4], color, 1, 8);
}
//绘制多边形
void drawConvexHull(cv::Mat& img, std::vector<cv::Point2f> points, Scalar color)
{
    for (size_t j = 0; j < points.size() - 1; j++)
        line(img, points[j], points[j + 1], color, 1, 8);
    line(img, points[0], points[points.size() - 1], color, 1, 8);
}

}
