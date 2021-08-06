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
#include "rm_util/math.hpp"

using namespace std;
using namespace cv;
namespace rm_util {

//0-180,90为垂直
float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2)
{
    float angle;
    if (point1.x == point2.x) {
        return CV_PI/2;
    } else {
        double k;
        k = -(point1.y - point2.y) / (point1.x - point2.x); //符号取反，图像坐标系和实际坐标系不统一
        angle = (float)atan(k);
    }
    if (angle < 0) {
        angle = angle + CV_PI;
    }
    return angle;
}

float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2)
{
    float a, b, c; //求角C
    float angle_c;
    a = cv::norm(vertex_point - point1);
    b = cv::norm(vertex_point - point2);
    c = cv::norm(point1 - point2);
    angle_c = static_cast<float>(acos((a * a + b * b - c * c) / (2 * a * b)));
    return angle_c;
}

bool calc_circle_from_3points(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f& point, float& r)
{
    double a, b, c, d, e, f;
    a = 2 * (p2.x - p1.x);
    b = 2 * (p2.y - p1.y);
    c = p2.x * p2.x + p2.y * p2.y - p1.x * p1.x - p1.y * p1.y;
    d = 2 * (p3.x - p2.x);
    e = 2 * (p3.y - p2.y);
    f = p3.x * p3.x + p3.y * p3.y - p2.x * p2.x - p2.y * p2.y;
    point.x = (b * f - e * c) / (b * d - e * a);
    point.y = (d * c - a * f) / (b * d - e * a);
    r = sqrt((point.x - p1.x) * (point.x - p1.x) + (point.y - p1.y) * (point.y - p1.y)); //半径
    return true;
}

}
