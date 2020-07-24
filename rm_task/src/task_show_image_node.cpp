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
#include "rm_task/task_show_image_node.h"

using namespace cv;
using namespace std;
using namespace rm_task;

TaskShowImageNode::TaskShowImageNode(std::string node_name,std::string conf_path)
                        :TaskImageProcNode(node_name){
    //set conf path
    conf_path_ = conf_path;
    start("/top_camera/image_raw");
    setRunFlag(true);
    cout<<"task show image init"<<endl;
}


void TaskShowImageNode::taskImageProcess(cv::Mat& img,double img_stamp){
    imshow("show_img",img);
    waitKey(1);
    cout<<"task show image,get image:"<<img_stamp<<endl;
}
void TaskShowImageNode::taskImageWait(){
    cout<<"task show image,wait get image"<<endl;
}
void TaskShowImageNode::taskSleep(){
    
    cout<<"task show image,sleep......"<<endl;
}

