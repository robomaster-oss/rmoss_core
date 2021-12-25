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
CamClient::CamClient(
  rclcpp::Node::SharedPtr node, std::string camera_name, Callback process_fn, bool spin_thread)
: node_(node)
{
  (void)spin_thread;
  connect(camera_name, process_fn);
}

CamClient::~CamClient()
{
  if (executor_) {
    executor_->cancel();
    executor_thread_->join();
  }
}

bool CamClient::connect(const std::string & camera_name, Callback cb)
{
  if (camera_name_ == camera_name) {
    RCLCPP_ERROR(node_->get_logger(), "camera %s is already connected.", camera_name.c_str());
    return false;
  }
  // try to cancel the prevoius camera
  disconnect();
  // set new camera
  camera_name_ = camera_name;
  auto img_cb = [cb](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      auto img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      cb(img, msg->header.stamp);
    };
  auto sub_opt = rclcpp::SubscriptionOptions();
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sub_opt.callback_group = callback_group_;
  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    camera_name + "/image_raw", 1, img_cb, sub_opt);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
  executor_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
  return true;
}

void CamClient::disconnect()
{
  if (img_sub_) {
    // cancel the prevoius camera
    executor_->cancel();
    executor_thread_->join();
    executor_.reset();
    executor_thread_.reset();
    img_sub_.reset();
    camera_name_ = "";
  }
}

bool CamClient::get_camera_info(sensor_msgs::msg::CameraInfo & info)
{
  if (camera_name_ == "") {
    RCLCPP_ERROR(node_->get_logger(), "[get_camera_info] camera should be connected!");
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
