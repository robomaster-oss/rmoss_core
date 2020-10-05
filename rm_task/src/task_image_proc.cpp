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
#include <cv_bridge/cv_bridge.h>
#include "rm_task/task_image_proc.h"

using namespace cv;
using namespace std;
using namespace rm_task;

TaskImageProc::TaskImageProc(rclcpp::Node::SharedPtr &nh){
    //init flag
    run_flag_=false;
    initflag_=false;
    get_img_flag_=false;
    nh_ = nh;
    //
    std::string topic_name = nh_->declare_parameter("cam_topic_name", "camera/image_raw");
    //create image subscriber
    img_sub_= image_transport::create_subscription(nh_.get(), topic_name, std::bind(
      &TaskImageProc::imgSubCb, this, std::placeholders::_1), "raw");
    //task thread
    task_thread_= std::thread(&TaskImageProc::mainTask, this);
}


void TaskImageProc::imgSubCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    if(run_flag_){
       imgbuf_=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
       if(!get_img_flag_){
          img_=imgbuf_.clone();
          img_stamp_=msg->header.stamp.sec + 0.000000001 * msg->header.stamp.nanosec;
          get_img_flag_=true;
       }
    }
}

void TaskImageProc::mainTask() {
    //run task 
    while(rclcpp::ok()) {
        if(run_flag_){
            if(get_img_flag_){//表示获取到图片
                taskImageProcess(img_,img_stamp_);
                get_img_flag_=false;//处理完，置位
            }else{
                taskImageWait();
                std::this_thread::sleep_for( std::chrono::milliseconds(2));
            }
        }else{
            taskSleep();
            std::this_thread::sleep_for( std::chrono::milliseconds(100));//休眠挂起
        }  
    }
}

void TaskImageProc::setRunFlag(bool flag){
    run_flag_=flag;
}


