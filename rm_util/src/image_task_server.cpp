/*******************************************************************************
 *  Copyright (c) 2021 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <cv_bridge/cv_bridge.h>
#include "rm_util/image_task_server.hpp"

using namespace cv;
using namespace std;
using namespace rm_util;

ImageTaskServer::ImageTaskServer(rclcpp::Node::SharedPtr &node,std::string topic_name,
            Callback process_fn,bool spin_thread){
    node_ = node;
    process_fn_ = process_fn;
    //TODO: use a dedicated thread to spin image subscription callback (wait for ROS Galactic?)
    spin_thread_ = spin_thread;
    run_flag_=false;
    //create image subscriber
    img_sub_= image_transport::create_subscription(node_.get(), topic_name, std::bind(
      &ImageTaskServer::img_cb, this, std::placeholders::_1), "raw");
}

void ImageTaskServer::img_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    if(run_flag_){
      auto img=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      auto img_stamp=msg->header.stamp.sec + 0.000000001 * msg->header.stamp.nanosec;
      process_fn_(img,img_stamp);
    }
}


void ImageTaskServer::start(){
    run_flag_=true;
}

void ImageTaskServer::stop(){
    run_flag_=false;
}


