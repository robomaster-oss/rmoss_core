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

#ifndef RM_TASK_TASK_SHOW_IMAGE_H
#define RM_TASK_TASK_SHOW_IMAGE_H

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "rm_task/task_image_proc.h"

namespace rm_task {

    //
    class TaskShowImage : public TaskImageProc{
    public:
        TaskShowImage(rclcpp::Node::SharedPtr &nh);
        ~TaskShowImage(){};
    private:
        void taskImageProcess(cv::Mat& img,double img_stamp);
        void taskImageWait();
        void taskSleep();
    private:
        rclcpp::Node::SharedPtr nh_;
    };
}
#endif //RM_TASK_TASK_SHOW_IMAGE_H

