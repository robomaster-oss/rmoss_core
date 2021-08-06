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

#include "rm_util/image_task_server.hpp"

#include <cv_bridge/cv_bridge.h>

#include <string>
namespace rm_util
{
ImageTaskServer::ImageTaskServer(
  rclcpp::Node::SharedPtr & node, std::string topic_name, Callback process_fn, bool spin_thread)
{
  node_ = node;
  process_fn_ = process_fn;
  // TODO(gezp): use a dedicated thread to spin image subscription callback (wait for ROS Galactic?)
  spin_thread_ = spin_thread;
  run_flag_ = false;
  // create image subscriber
  img_sub_ = image_transport::create_subscription(
    node_.get(), topic_name, std::bind(&ImageTaskServer::img_cb, this, std::placeholders::_1),
    "raw");
}

void ImageTaskServer::img_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (run_flag_) {
    auto img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    auto img_stamp = msg->header.stamp.sec + 0.000000001 * msg->header.stamp.nanosec;
    process_fn_(img, img_stamp);
  }
}

void ImageTaskServer::start() {run_flag_ = true;}

void ImageTaskServer::stop() {run_flag_ = false;}

}  // namespace rm_util
