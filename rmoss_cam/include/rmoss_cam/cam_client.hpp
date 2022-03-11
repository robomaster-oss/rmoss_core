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

#ifndef RMOSS_CAM__CAM_CLIENT_HPP_
#define RMOSS_CAM__CAM_CLIENT_HPP_

#include <string>
#include <thread>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_server_manager.hpp"

namespace rmoss_cam
{

// camera client wrapper to subscribe image (used by auto aim task, power rune task..)
class CamClient
{
public:
  typedef std::function<void (const cv::Mat &, const rclcpp::Time &)> Callback;
  explicit CamClient(rclcpp::Node::SharedPtr node)
  : node_(node) {}
  ~CamClient();

  void set_camera_name(const std::string & camera_name);
  void set_cam_server_manager(std::shared_ptr<CamServerManager> manager);

  virtual bool connect(Callback cb);
  virtual void disconnect();
  bool is_connect() {return is_connected_;}
  bool get_camera_info(sensor_msgs::msg::CameraInfo & info);

protected:
  rclcpp::Node::SharedPtr node_;
  std::string camera_name_;
  // capture image by topic
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> executor_thread_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;  // 订阅图片数据
  bool is_connected_{false};
  // for intra-comms
  bool use_intra_comms_{false};
  std::shared_ptr<CamServerManager> cam_server_manager_;
  std::mutex mut_;
  std::condition_variable cond_;
  cv::Mat img_;
  rclcpp::Time stamp_;
  std::unique_ptr<std::thread> callback_thread_;
  std::shared_ptr<CamServer> cur_server_;
  int cur_server_cb_idx_{-1};
};
}  // namespace rmoss_cam

#endif  // RMOSS_CAM__CAM_CLIENT_HPP_
