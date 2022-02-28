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

#include "rmoss_cam/intra_cam_client.hpp"

#include <string>
#include <memory>

namespace rmoss_cam
{

bool IntraCamClient::connect(const std::string & camera_name, Callback cb)
{
  if (camera_name_ == camera_name) {
    RCLCPP_ERROR(node_->get_logger(), "camera %s is already connected.", camera_name.c_str());
    return false;
  }
  auto server = cam_server_manager_->get_cam_server(camera_name);
  if (!server) {
    RCLCPP_ERROR(node_->get_logger(), "failed to find camera server %s.", camera_name.c_str());
    return false;
  }
  // disconnect the previous camera
  disconnect();
  // set new camera
  cur_server_ = server;
  camera_name_ = camera_name;
  ok_ = true;
  callback_thread_ = std::make_unique<std::thread>(
    [this, cb]() {
      while (ok_) {
        std::unique_lock<std::mutex> lock(mut_);
        cond_.wait(lock);
        if (ok_) {
          cb(img_, stamp_);
        }
      }
    });
  cur_server_cb_idx_ = cur_server_->add_callback(
    [this](const cv::Mat & img, const rclcpp::Time & stamp) {
      if (mut_.try_lock()) {
        img.copyTo(img_);
        stamp_ = stamp;
        mut_.unlock();
        cond_.notify_one();
      }
    });
  return true;
}

void IntraCamClient::disconnect()
{
  if (ok_) {
    ok_ = false;
    cur_server_->remove_callback(cur_server_cb_idx_);
    cond_.notify_one();
    callback_thread_->join();
    camera_name_ = "";
  }
}

IntraCamClient::~IntraCamClient()
{
  if (ok_) {
    ok_ = false;
    cur_server_->remove_callback(cur_server_cb_idx_);
    cond_.notify_one();
    callback_thread_->join();
  }
}

}  // namespace rmoss_cam
