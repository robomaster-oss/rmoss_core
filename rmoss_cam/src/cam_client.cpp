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

#include "rmoss_cam/cam_client.hpp"

#include <string>
#include <memory>

#include "rmoss_interfaces/srv/get_camera_info.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

namespace rmoss_cam
{

CamClient::~CamClient()
{
  if (is_connected_) {
    disconnect();
  }
}

void CamClient::set_camera_name(const std::string & camera_name)
{
  if (is_connected_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[set_camera_name] camera %s is already connected, please use disconnect().",
      camera_name_.c_str());
    return;
  }
  camera_name_ = camera_name;
}

void CamClient::set_camera_callback(Callback cb)
{
  if (is_connected_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[set_camera_callback] camera %s is already connected, please use disconnect().",
      camera_name_.c_str());
    return;
  }
  cb_ = cb;
}

bool CamClient::connect()
{
  // check
  if (is_connected_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[connect] camera %s is already connected, please use disconnect().",
      camera_name_.c_str());
    return false;
  }
  if (camera_name_ == "") {
    RCLCPP_ERROR(node_->get_logger(), "[connect] camera_name is invaild!");
    return false;
  }
  if (!cb_) {
    RCLCPP_ERROR(node_->get_logger(), "[connect] callback is invaild!");
    return false;
  }
  // connect
  auto img_cb = [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      auto img =
        cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()));
      cb_(img, msg->header.stamp);
    };
  auto sub_opt = rclcpp::SubscriptionOptions();
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sub_opt.callback_group = callback_group_;
  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    camera_name_ + "/image_raw", 1, img_cb, sub_opt);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
  executor_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});

  is_connected_ = true;
  return true;
}

void CamClient::disconnect()
{
  if (is_connected_) {
    // cancel the prevoius camera
    executor_->cancel();
    executor_thread_->join();
    executor_.reset();
    executor_thread_.reset();
    img_sub_.reset();
    camera_name_ = "";
    is_connected_ = false;
  }
}

bool CamClient::get_camera_info(sensor_msgs::msg::CameraInfo & info)
{
  if (camera_name_ == "") {
    RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] camera_name is invaild!");
    return false;
  }
  auto callback_group = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto client = node_->create_client<rmoss_interfaces::srv::GetCameraInfo>(
    camera_name_ + "/get_camera_info",
    rmw_qos_profile_services_default,
    callback_group);
  exec->add_callback_group(callback_group, node_->get_node_base_interface());
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] client interrupted.");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[get_camera_info] waiting for service.");
  }
  auto request = std::make_shared<rmoss_interfaces::srv::GetCameraInfo::Request>();
  auto result_future = client->async_send_request(request);
  if (exec->spin_until_future_complete(result_future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] waiting for service failed.");
    return false;
  }
  auto result = result_future.get();
  if (result->success) {
    info = result->camera_info;
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] service call failed.");
    return false;
  }
}

}  // namespace rmoss_cam
