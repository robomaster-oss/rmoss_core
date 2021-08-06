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
#ifndef RM_TASK_TASK_IMAGE_PROC_HPP
#define RM_TASK_TASK_IMAGE_PROC_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>
#include <string>

namespace rm_util {
    typedef std::function<void(cv::Mat&, double)> Callback;
    //图像处理相关任务基类.如自动瞄准任务，能量机关任务
    class ImageTaskServer
    {
    public:
        ImageTaskServer() = delete;
        explicit ImageTaskServer(
            rclcpp::Node::SharedPtr &node,
            std::string topic_name,
            Callback process_fn,
            bool spin_thread=true);
        ~ImageTaskServer(){}; 

        void start();
        void stop();
    private:
        void img_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    private:
        rclcpp::Node::SharedPtr node_;
        image_transport::Subscriber img_sub_;//订阅图片数据
        bool spin_thread_;
        std::unique_ptr<std::thread> task_thread_;
        Callback process_fn_;
        bool run_flag_; //运行标志位
    };
}

#endif //RM_TASK_TASK_IMAGE_PROC_HPP




