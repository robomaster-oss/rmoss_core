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

#include "rm_cam/cam_server.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace rm_cam
{

constexpr const CamParamType kCamParamTypes[] = {
  CamParamType::Width,
  CamParamType::Height,
  CamParamType::AutoExposure,
  CamParamType::Exposure,
  CamParamType::Brightness,
  CamParamType::AutoWhiteBalance,
  CamParamType::WhiteBalance,
  CamParamType::Gain,
  CamParamType::Gamma,
  CamParamType::Contrast,
  CamParamType::Saturation,
  CamParamType::Hue,
  CamParamType::Fps
};

constexpr const char * kCamParamTypeNames[] = {
  "width",
  "height",
  "auto_exposure",
  "exposure",
  "brightness",
  "auto_white_balance",
  "white_balance",
  "gain",
  "gamma",
  "contrast",
  "saturation",
  "hue",
  "fps"
};

CamServer::CamServer(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<CamInterface> cam_intercace)
: node_(node), cam_intercace_(cam_intercace)
{
  // 相机参数获取并设置，配置文件中的值会覆盖默认值
  int data;
  constexpr int param_num = sizeof(kCamParamTypes) / sizeof(CamParamType);
  for (int i = 0; i < param_num; i++) {
    if (cam_intercace_->get_parameter(kCamParamTypes[i], data)) {
      node->declare_parameter(kCamParamTypeNames[i], data);
      node->get_parameter(kCamParamTypeNames[i], data);
      cam_intercace_->set_parameter(kCamParamTypes[i], data);
    }
  }
  // 记录fps
  cam_intercace_->get_parameter(rm_cam::CamParamType::Fps, fps_);
  // 打开摄像头
  if (!cam_intercace_->open()) {
    RCLCPP_ERROR(node_->get_logger(), "fail to open camera!");
    return;
  }
  // fps比较特殊，如果fps没有被设置，或值非法，则设置为默认值30
  if (fps_ <= 0) {
    fps_ = 30;
    cam_intercace_->set_parameter(rm_cam::CamParamType::Fps, 30);
  }
  // declare parameters
  node->declare_parameter("camera_name", "camera");
  node->declare_parameter("camera_k", rclcpp::ParameterValue(std::vector<double>()));
  node->declare_parameter("camera_d", rclcpp::ParameterValue(std::vector<double>()));
  node->declare_parameter("camera_p", rclcpp::ParameterValue(std::vector<double>()));
  // get parameters
  auto camera_name = node->get_parameter("camera_name").as_string();
  node->get_parameter("camera_k", camera_k_);
  if (!camera_k_.empty() && camera_k_.size() != 9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "the size of the camera intrinsic parameter should be 9");
  }
  node->get_parameter("camera_d", camera_d_);
  node->get_parameter("camera_p", camera_p_);
  if (!camera_p_.empty() && camera_p_.size() != 12) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "the size of the camera extrinsic parameter should be 12");
  }
  // create image publisher
  img_pub_ = image_transport::create_publisher(node.get(), camera_name + "/image_raw");
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / fps_));
  timer_ = node->create_wall_timer(period_ms, std::bind(&CamServer::timer_callback, this));
  // create GetCameraInfo service
  using namespace std::placeholders;
  camera_info_service_ = node->create_service<rmoss_interfaces::srv::GetCameraInfo>(
    camera_name + "/get_camera_info",
    std::bind(&CamServer::camera_info_callback, this, _1, _2, _3));
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
}

void CamServer::timer_callback()
{
  if (cam_intercace_->grab_image(img_)) {
    auto header = std_msgs::msg::Header();
    header.stamp = node_->now();
    // publish image msg
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
      header, "bgr8", img_).toImageMsg();
    img_pub_.publish(img_msg);
  } else {
    // try to reopen camera
    if (reopen_cnt % fps_ == 0) {
      cam_intercace_->close();
      std::this_thread::sleep_for(100ms);
      if (cam_intercace_->open()) {
        RCLCPP_INFO(node_->get_logger(), "reopen camera successed!");
      } else {
        RCLCPP_INFO(node_->get_logger(), "reopen camera failed!");
      }
    }
    reopen_cnt++;
  }
}

void CamServer::camera_info_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const rmoss_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
  rmoss_interfaces::srv::GetCameraInfo::Response::SharedPtr response)
{
  (void)request_header;
  (void)request;
  auto & camera_info = response->camera_info;
  int data;
  cam_intercace_->get_parameter(rm_cam::CamParamType::Height, data);
  camera_info.height = data;
  cam_intercace_->get_parameter(rm_cam::CamParamType::Width, data);
  camera_info.width = data;
  std::copy_n(camera_k_.begin(), 9, camera_info.k.begin());
  camera_info.d = camera_d_;
  response->success = true;
}


}  // namespace rm_cam
