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
#include "rm_common/math.hpp"

using namespace std;
using namespace cv;
namespace rm_common {

//0-180,90为垂直
float calcInclineAngle(cv::Point2f point1, cv::Point2f point2)
{
    float angle;
    if (point1.x == point2.x) {
        return 90;
    } else {
        double k;
        k = -(point1.y - point2.y) / (point1.x - point2.x); //符号取反，图像坐标系和实际坐标系不统一
        angle = (float)(atan(k) * 180 / CV_PI);
    }
    if (angle < 0) {
        angle = angle + 180;
    }
    return angle;
}

float calcInnerAngle(cv::Point2f vertexPoint, cv::Point2f point1, cv::Point2f point2)
{
    float a, b, c; //求角C
    float angleC;
    a = cv::norm(vertexPoint - point1);
    b = cv::norm(vertexPoint - point2);
    c = cv::norm(point1 - point2);
    angleC = static_cast<float>(acos((a * a + b * b - c * c) / (2 * a * b)) / CV_PI * 180);
    return angleC;
}

bool calcCircle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f& point, float& r)
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
