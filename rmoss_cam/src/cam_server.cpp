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

#include "rmoss_cam/cam_server.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace rmoss_cam
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
  CamParamType::RGain,
  CamParamType::GGain,
  CamParamType::BGain,
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
  "Rgain",
  "Ggain",
  "Bgain",
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
  std::string camera_info_url = "";
  // declare parameters
  node->declare_parameter("camera_name", camera_name_);
  node->declare_parameter("frame_id", camera_frame_id_);
  node->declare_parameter("camera_info_url", camera_info_url);
  node->declare_parameter("autostart", run_flag_);
  node->declare_parameter("use_sensor_data_qos", use_qos_profile_sensor_data_);
  node->declare_parameter(
    "use_image_transport_camera_publisher",
    use_image_transport_camera_publisher_);
  node->get_parameter("camera_name", camera_name_);
  node->get_parameter("frame_id", camera_frame_id_);
  node->get_parameter("camera_info_url", camera_info_url);
  node->get_parameter("autostart", run_flag_);
  node->get_parameter("use_sensor_data_qos", use_qos_profile_sensor_data_);
  node->get_parameter(
    "use_image_transport_camera_publisher",
    use_image_transport_camera_publisher_);
  if (camera_frame_id_ == "") {
    camera_frame_id_ = camera_name_ + "_optical";
  }
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
  // 获取fps
  cam_intercace_->get_parameter(rmoss_cam::CamParamType::Fps, fps_);
  // 如果fps值非法，则设置为默认值30
  if (fps_ <= 0) {
    fps_ = 30;
    cam_intercace_->set_parameter(rmoss_cam::CamParamType::Fps, 30);
  }
  // 打开摄像头
  if (!cam_intercace_->open()) {
    RCLCPP_FATAL(
      node_->get_logger(), "fail to open camera: %s",
      cam_intercace_->error_message().c_str());
  }
  // create camera info manager
  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    node_.get(), camera_name_, camera_info_url);
  if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
    RCLCPP_INFO(
      node_->get_logger(), "Calibration calibrated from file '%s'", camera_info_url.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Calibration file '%s' is missing", camera_info_url.c_str());
  }
  cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());

  // create image_transport
  if (!use_image_transport_camera_publisher_) {
    img_pub_ = std::make_shared<image_transport::Publisher>(
      image_transport::create_publisher(
        node_.get(),
        camera_name_ + "/image_raw",
        use_qos_profile_sensor_data_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  } else {
    cam_pub_ = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
        node_.get(),
        camera_name_ + "/image_raw",
        use_qos_profile_sensor_data_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  }
  // init grab image timer
  init_timer();

  // create GetCameraInfo service
  using namespace std::placeholders;
  get_camera_info_srv_ = node->create_service<rmoss_interfaces::srv::GetCameraInfo>(
    camera_name_ + "/get_camera_info",
    std::bind(&CamServer::get_camera_info_cb, this, _1, _2));
  init_task_manager();
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
}

void CamServer::init_timer()
{
  std::function<void()> timer_callback;
  if (!use_image_transport_camera_publisher_) {
    timer_callback = [this]() {
        if (!run_flag_) {
          return;
        }
        if (cam_intercace_->grab_image(img_)) {
          cam_status_ok_ = true;
          rclcpp::Time stamp = node_->now();
          // publish image msg
          if (this->img_pub_->getNumSubscribers() > 0) {
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = camera_frame_id_;
            msg->encoding = "bgr8";
            msg->width = img_.cols;
            msg->height = img_.rows;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(img_.step);
            msg->is_bigendian = false;
            msg->data.assign(img_.datastart, img_.dataend);
            img_pub_->publish(std::move(msg));
          }
        } else {
          // try to reopen camera
          cam_status_ok_ = false;
          if (reopen_cnt % fps_ == 0) {
            cam_intercace_->close();
            std::this_thread::sleep_for(100ms);
            if (cam_intercace_->open()) {
              RCLCPP_WARN(node_->get_logger(), "reopen camera successed!");
            } else {
              RCLCPP_WARN(node_->get_logger(), "reopen camera failed!");
            }
          }
          reopen_cnt++;
        }
      };
  } else {
    timer_callback = [this]() {
        if (!run_flag_) {
          return;
        }
        if (cam_intercace_->grab_image(img_)) {
          cam_status_ok_ = true;
          rclcpp::Time stamp = node_->now();
          // publish image msg
          if (this->cam_pub_->getNumSubscribers() > 0) {
            sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = camera_frame_id_;
            msg->encoding = "bgr8";
            msg->width = img_.cols;
            msg->height = img_.rows;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(img_.step);
            msg->is_bigendian = false;
            msg->data.assign(img_.datastart, img_.dataend);
            cam_info_->header.stamp = stamp;
            cam_info_->header.frame_id = camera_frame_id_;
            cam_pub_->publish(*msg, *cam_info_);
          }
        } else {
          // try to reopen camera
          cam_status_ok_ = false;
          if (reopen_cnt % fps_ == 0) {
            cam_intercace_->close();
            std::this_thread::sleep_for(100ms);
            if (cam_intercace_->open()) {
              RCLCPP_WARN(node_->get_logger(), "reopen camera successed!");
            } else {
              RCLCPP_WARN(node_->get_logger(), "reopen camera failed!");
            }
          }
          reopen_cnt++;
        }
      };
  }
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / fps_));
  timer_ = node_->create_wall_timer(period_ms, timer_callback);
}

void CamServer::init_task_manager()
{
  // task manager
  auto get_task_status_cb = [&]() {
      if (run_flag_) {
        if (!cam_status_ok_) {
          return rmoss_util::TaskStatus::Error;
        }
        return rmoss_util::TaskStatus::Running;
      } else {
        if (!cam_intercace_->is_open()) {
          return rmoss_util::TaskStatus::Error;
        }
        return rmoss_util::TaskStatus::Idle;
      }
    };
  auto control_task_cb = [&](rmoss_util::TaskCmd cmd) {
      if (cmd == rmoss_util::TaskCmd::Start) {
        run_flag_ = true;
      } else if (cmd == rmoss_util::TaskCmd::Stop) {
        run_flag_ = false;
      } else {
        return false;
      }
      return true;
    };
  task_manager_ = std::make_shared<rmoss_util::TaskManager>(
    node_, get_task_status_cb, control_task_cb);
}

void CamServer::get_camera_info_cb(
  const rmoss_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
  rmoss_interfaces::srv::GetCameraInfo::Response::SharedPtr response)
{
  (void)request;
  if (camera_info_manager_->isCalibrated()) {
    response->camera_info = camera_info_manager_->getCameraInfo();
    response->success = true;
  } else {
    response->success = false;
  }
}

}  // namespace rmoss_cam
