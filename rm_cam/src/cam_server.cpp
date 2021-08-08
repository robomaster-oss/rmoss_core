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

CamServer::CamServer(rclcpp::Node::SharedPtr node, std::shared_ptr<CamInterface> cam_intercace)
: node_(node), cam_intercace_(cam_intercace)
{
  // declare parameters
  node_->declare_parameter("camera_name", "camera");
  node->declare_parameter("image_width", 640);
  node->declare_parameter("image_height", 480);
  node->declare_parameter("Fps", 30);
  node_->declare_parameter("camera_matrix", rclcpp::ParameterValue(std::vector<double>()));
  node_->declare_parameter("camera_distortion", rclcpp::ParameterValue(std::vector<double>()));
  // get parameters
  auto camera_name = node_->get_parameter("camera_name").as_string();
  auto width = node_->get_parameter("image_width").as_int();
  auto height = node_->get_parameter("image_height").as_int();
  auto Fps = node_->get_parameter("Fps").as_int();
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / Fps));
  node_->get_parameter("camera_matrix", camera_matrix_);
  if (camera_matrix_.size() != 9) {
    RCLCPP_WARN(node_->get_logger(), "camera matrix is invalid! size: %d", camera_matrix_.size());
  }
  node_->get_parameter("camera_distortion", camera_distortion_);
  if (camera_distortion_.size() != 5) {
    RCLCPP_WARN(
      node_->get_logger(), "camera distortion is invalid! size: %d", camera_distortion_.size());
  }
  // set camera
  cam_intercace_->set_parameter(rm_cam::CamParamType::Width, width);
  cam_intercace_->set_parameter(rm_cam::CamParamType::Height, height);
  cam_intercace_->set_parameter(rm_cam::CamParamType::Fps, Fps);
  cam_intercace_->open();
  // create image publisher
  img_pub_ = image_transport::create_publisher(node_.get(), camera_name + "/image_raw");
  timer_ = node_->create_wall_timer(period_ms, std::bind(&CamServer::timer_callback, this));
  // create GetCameraInfo service
  using namespace std::placeholders;
  camera_info_service_ = node_->create_service<rmoss_interfaces::srv::GetCameraInfo>(
    camera_name + "/get_camera_info",
    std::bind(&CamServer::camera_info_callback, this, _1, _2, _3));
}

void CamServer::timer_callback()
{
  if (cam_intercace_->grab_image(img_)) {
    // publish image msg
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", img_).toImageMsg();
    img_msg->header.frame_id = "camera";
    img_msg->header.stamp = rclcpp::Clock().now();
    img_pub_.publish(img_msg);
  } else {
    // try to reopen camera
    if (reopen_cnt % 100 == 0) {
      RCLCPP_INFO(node_->get_logger(), "Reopen Camera!");
      cam_intercace_->close();
      std::this_thread::sleep_for(100ms);
      cam_intercace_->open();
      reopen_cnt++;
    }
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
  if (camera_matrix_.size() != 9 || camera_distortion_.size() != 5) {
    response->success = false;
    return;
  }
  std::copy_n(camera_matrix_.begin(), 9, camera_info.k.begin());
  camera_info.d = camera_distortion_;
  response->success = true;
}


}  // namespace rm_cam
