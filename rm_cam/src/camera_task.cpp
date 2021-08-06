// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "rm_cam/camera_task.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

using namespace cv;
using namespace std;
using namespace rm_cam;

CameraTask::CameraTask(rclcpp::Node::SharedPtr& nh, std::shared_ptr<CamDevInterface> cam_intercace)
{
    //init
    run_flag_ = false;
    nh_ = nh;
    cam_intercace_ = cam_intercace;
    //set fps
    int fps;
    if (cam_intercace_->getParameter(CamParameter::fps, fps)) {
        fps_period_us_ = 1000000.0 / fps;
    } else {
        fps_period_us_ = 1;
    }
    //create image publisher
    std::string topic_name = nh_->declare_parameter("cam_topic_name", "camera/image_raw");
    img_pub_ = image_transport::create_publisher(nh_.get(), topic_name);
    // start cam thread
    cam_thread_ = std::thread(&CameraTask::capThread, this);
    // start the camera
    run_flag_ = true;
    RCLCPP_INFO(nh_->get_logger(), "cam thread start.");
}

void CameraTask::capThread()
{
    cv::Mat img;
    rclcpp::Time cap_start;
    int64_t fps_cap_us;
    double time_diff_ms;
    while (rclcpp::ok()) {
        if (run_flag_) {
            cap_start = rclcpp::Clock().now();
            if (cam_intercace_->capImg(img)) {
                //publish msg
                sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                img_msg->header.frame_id = "camera";
                img_msg->header.stamp = cap_start;
                img_pub_.publish(img_msg);
                // fps control
                fps_cap_us = (rclcpp::Clock().now().nanoseconds() - cap_start.nanoseconds()) / 1000;
                time_diff_ms = 0.001 * (fps_period_us_ - fps_cap_us);
                if (time_diff_ms > 1000) { //>1000ms
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)time_diff_ms));
                } else if (time_diff_ms > 0) { // 0-1000ms
                    std::this_thread::sleep_for(std::chrono::microseconds((int)(time_diff_ms * 1000)));
                } else {
                    // pass
                }
            } else {
                RCLCPP_INFO(nh_->get_logger(), "capThread():cap err!");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            //休眠挂起
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    RCLCPP_INFO(nh_->get_logger(), "capThread():cap thread exit!");
}
