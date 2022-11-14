// Copyright 2022 RoboMaster-OSS
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_cam/image_task_demo_node.hpp"

using namespace std::chrono_literals;

namespace rmoss_cam
{

ImageTaskDemoNode::ImageTaskDemoNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("image_task_demo", options);
  // parameters
  node_->declare_parameter("camera_name", camera_name_);
  node_->declare_parameter("info_fps", info_fps_);
  node_->declare_parameter("process_time_ms", process_time_ms_);
  node_->get_parameter("camera_name", camera_name_);
  node_->get_parameter("info_fps", info_fps_);
  node_->get_parameter("process_time_ms", process_time_ms_);
  // cam client
  cam_client_ = std::make_shared<rmoss_cam::CamClient>(node_);
  // timer for init
  init_timer_ = node_->create_wall_timer(
    0s, [this]() {
      init_timer_->cancel();
      this->init();
    });
}

void ImageTaskDemoNode::init()
{
  cam_client_->set_camera_name(camera_name_);
  cam_client_->set_camera_callback(
    [this](const cv::Mat & img, const rclcpp::Time & stamp) {
      this->process(img, stamp);
    });
  // wait to connect camera
  bool ret = false;
  while (!ret) {
    ret = cam_client_->connect();
    if (!ret) {
      RCLCPP_WARN(node_->get_logger(), "wait 1s to open camera.");
      std::this_thread::sleep_for(1s);
    }
  }
  RCLCPP_WARN(node_->get_logger(), "init sucessfully!");
}

void ImageTaskDemoNode::process(const cv::Mat & /*img*/, const rclcpp::Time & stamp)
{
  if (cnt_ == 0) {
    start_time_ = node_->get_clock()->now().seconds();
  }
  if (cnt_ < info_fps_) {
    cnt_++;
    auto delay = (node_->get_clock()->now().seconds() - stamp.seconds());
    total_delay_ = total_delay_ + delay;
  } else {
    // info
    auto last_time = node_->get_clock()->now().seconds();
    auto recv_fps = 1.0 * info_fps_ / (last_time - start_time_);
    auto recv_delay = total_delay_ / info_fps_;
    RCLCPP_INFO(node_->get_logger(), "fps: %f, delay: %f", recv_fps, recv_delay);
    // reset
    cnt_ = 0;
    total_delay_ = 0;
  }
  // process image
  std::this_thread::sleep_for(std::chrono::milliseconds(process_time_ms_));
}

}  // namespace rmoss_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_cam::ImageTaskDemoNode)
