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

#ifndef RMOSS_CAM__INTRA_CAM_CLIENT_HPP_
#define RMOSS_CAM__INTRA_CAM_CLIENT_HPP_

#include <string>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_cam/cam_client.hpp"
#include "rmoss_cam/cam_server_manager.hpp"

namespace rmoss_cam
{

// camera client wrapper to subscribe image (used by auto aim task, power rune task..)
class IntraCamClient : public CamClient
{
public:
  typedef std::function<void (const cv::Mat &, const rclcpp::Time &)> Callback;
  explicit IntraCamClient(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<CamServerManager> manager = nullptr)
  : CamClient(node), cam_server_manager_(manager) {}
  ~IntraCamClient();

  bool connect(const std::string & camera_name, Callback cb) override;
  void disconnect() override;

  void add_cam_server(std::shared_ptr<CamServer> cam_server);

private:
  std::shared_ptr<CamServerManager> cam_server_manager_;
  // callback data
  std::mutex mut_;
  std::condition_variable cond_;
  cv::Mat img_;
  rclcpp::Time stamp_;
  std::unique_ptr<std::thread> callback_thread_;
  bool ok_{false};
  std::shared_ptr<CamServer> cur_server_;
  int cur_server_cb_idx_{-1};
};
}  // namespace rmoss_cam

#endif  // RMOSS_CAM__INTRA_CAM_CLIENT_HPP_
