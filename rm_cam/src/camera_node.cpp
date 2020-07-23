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
#include "rm_cam/camera_node.h"
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace rm_cam;

CameraNode::CameraNode(std::string node_name): rclcpp::Node(node_name)
{
    run_flag_ = false;  
}

CameraNode::~CameraNode() {}

int CameraNode::init(CamDevInterface *cam_intercace,std::string topic_name) {
    cam_intercace_ = cam_intercace;
    //set fps
    int fps;
    if(cam_intercace_->getParameter(Fps,fps)){
        fps_period_us_ = 1000000.0 / fps;
    }else{
        fps_period_us_=1;
    }
    //create image publisher
    img_pub_ = image_transport::create_publisher(this, topic_name);
    // start cam thread
    cam_thread_ = std::thread(&CameraNode::capThread, this);
    // start the camera
    run_flag_ = true;
    RCLCPP_INFO(this->get_logger(), "init():cam thread start.");
    return 0;
}

void CameraNode::capThread() {
    cv::Mat img;
    rclcpp::Time cap_start;
    int64_t fps_cap_us;
    double time_diff_ms;
    while (rclcpp::ok()){
        if (run_flag_) {
            cap_start = rclcpp::Clock().now();
            if (cam_intercace_->capImg(img) == 0) {
                //publish msg
                sensor_msgs::msg::Image::SharedPtr img_msg =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
                        .toImageMsg();
                img_msg->header.frame_id = "camera";
                img_msg->header.stamp = cap_start;
                img_pub_.publish(img_msg);
                // fps control
                fps_cap_us =( rclcpp::Clock().now().nanoseconds() - cap_start.nanoseconds()) / 1000;
                time_diff_ms = 0.001* (fps_period_us_ - fps_cap_us);
                if (time_diff_ms > 1000) {  //>1000ms
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds((int)time_diff_ms));
                } else if (time_diff_ms > 0) {  // 0-1000ms
                    std::this_thread::sleep_for(std::chrono::microseconds(
                        (int)(time_diff_ms  * 1000));
                } else {
                    // pass
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "capThread():cap err!");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            //休眠挂起
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    RCLCPP_INFO(this->get_logger(), "capThread():cap thread exit!");
}

