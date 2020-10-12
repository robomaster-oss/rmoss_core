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
#include "rm_tool/image_tool.hpp"

using namespace std;
using namespace cv;
using namespace rm_tool;


float ImageTool::calc2PointDistance(cv::Point2f point1, cv::Point2f point2) {
    return sqrt(((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)));
}

float ImageTool::calc2PointDistance(cv::Point3f point1, cv::Point3f point2) {
    return sqrt((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)+(point1.z-point2.z)*(point1.z-point2.z));
}

//0-180,90为垂直
float ImageTool::calc2PointAngle(cv::Point2f point1, cv::Point2f point2) {
    float angle;
    if(point1.x == point2.x){
        return 90;
    } else{
        double k;
        k= -(point1.y - point2.y) / (point1.x - point2.x);//符号取反，图像坐标系和实际坐标系不统一
        angle=(float)(atan(k)*180/CV_PI);
    }
    if(angle<0){
         angle=angle+180;
    }
     return angle;

}

float ImageTool::calcTriangleInnerAngle(cv::Point2f vertexPoint, cv::Point2f point1, cv::Point2f point2) {
    float a,b,c;//求角C
    float angleC;
    a=calc2PointDistance(vertexPoint,point1);
    b=calc2PointDistance(vertexPoint,point2);
    c=calc2PointDistance(point1,point2);
    angleC=static_cast<float>(acos((a * a + b * b - c * c) / (2 * a * b)) / CV_PI * 180);
    return angleC;

}


