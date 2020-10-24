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
#include "rm_task/task_show_image.hpp"

using namespace cv;
using namespace std;
using namespace rm_task;

TaskShowImage::TaskShowImage(rclcpp::Node::SharedPtr &nh)
                        :TaskImageProc(nh){
    nh_ = nh;
    startTask();
    cout<<"task show image init"<<endl;
}


void TaskShowImage::taskImageProcess(cv::Mat& img,double img_stamp){
    imshow("show_img",img);
    waitKey(1);
    cout<<"task show image,get image:"<<img_stamp<<endl;
}
void TaskShowImage::taskImageWait(){
    cout<<"task show image,wait get image"<<endl;
}
void TaskShowImage::taskSleep(){
    cout<<"task show image,sleep......"<<endl;
}

