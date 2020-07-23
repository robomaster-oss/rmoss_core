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
#ifndef RM_TASK_TASK_IMAGE_PROC_NODE_H
#define RM_TASK_TASK_IMAGE_PROC_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>

namespace rm_task {
    //图像处理相关任务基类.如自动瞄准任务，能量机关任务
    class TaskImageProcNode: public rclcpp::Node
    {
    public:
        TaskImageProcNode(std::string node_name);
        ~TaskImageProcNode(){}; 
    private:
        void mainTask();
        void imgSubCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    protected:
        void setTopicName(std::string topic_name);
        void setRunFlag(bool flag);
        bool isInitOK();
    public:
        virtual int initTask()=0;
        virtual void taskImageProcess(cv::Mat& img,double img_stamp)=0;
        virtual void taskImageWait(){};
        virtual void taskSleep(){};
    private:
        //tool
        image_transport::Subscriber img_sub_;//订阅图片数据
        std::thread task_thread_;
        //data
        std::string topic_name_;
        cv::Mat imgbuf_, img_; //获取的图片，以及缓存图片
        bool initflag_;
        bool get_img_flag_;//使用flag实现多线程同步机制
        bool run_flag_; //运行标志位
        double img_stamp_;
    };
}

#endif //RM_TASK_TASK_IMAGE_PROC_NODE_H




