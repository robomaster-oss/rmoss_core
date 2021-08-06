// Copyright 2021 robomaster-oss.
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

#ifndef RM_UTIL__IMAGE_TASK_SERVER_HPP_
#define RM_UTIL__IMAGE_TASK_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <memory>

namespace rm_util
{
typedef std::function<void (cv::Mat &, double)> Callback;
// 图像处理相关任务基类.如自动瞄准任务，能量机关任务
class ImageTaskServer
{
public:
  ImageTaskServer() = delete;
  explicit ImageTaskServer(
    rclcpp::Node::SharedPtr & node, std::string topic_name, Callback process_fn,
    bool spin_thread = true);
  ~ImageTaskServer() {}

  void start();
  void stop();

private:
  void img_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber img_sub_;  // 订阅图片数据
  bool spin_thread_;
  std::unique_ptr<std::thread> task_thread_;
  Callback process_fn_;
  bool run_flag_;  // 运行标志位
};
}  // namespace rm_util

#endif  // RM_UTIL__IMAGE_TASK_SERVER_HPP_
