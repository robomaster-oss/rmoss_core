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
#include "rm_tool/debug_tool.h"
using namespace std;
using namespace cv;
using namespace rm_tool;

bool DebugTool::is_debug=false;
//调试部分
//绘制旋转矩形
Scalar DebugTool::getColor(int type){
     Scalar color;
    if(type==0){
        color = Scalar(255,0,0);
    }else if(type==1){
        color = Scalar(0,255,0);
    }else{
        color = Scalar(0,0,255);
    }
    return  color;
}


//绘制旋转矩形
void DebugTool::drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type){
    if(is_debug){
        Scalar color=getColor(type);
        Point2f rect_points[4];
        r.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( img, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
}
//绘制四边形
void DebugTool::draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type) {
    if(is_debug){
        Scalar color=getColor(type);
        for( int j = 0; j < 4; j++ )
            line( img, point2fs[j], point2fs[(j+1)%4], color, 1, 8 );
    }
}
//绘制多边形
void DebugTool::drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type){
    if(is_debug){
        Scalar color=getColor(type);
        for( size_t j = 0; j < points.size()-1; j++ )
            line( img, points[j], points[j+1], color, 1, 8 );
        line( img, points[0], points[points.size()-1], color, 1, 8 );
    }
}

