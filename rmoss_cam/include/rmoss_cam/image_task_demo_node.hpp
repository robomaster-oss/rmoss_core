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

#ifndef RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_
#define RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/cam_client.hpp"

namespace rmoss_cam
{
class ImageTaskDemoNode
{
public:
  explicit ImageTaskDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void init();
  void process(const cv::Mat & img, const rclcpp::Time & stamp);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<rmoss_cam::CamClient> cam_client_;
  std::string camera_name_;
  int cnt_{0};
  int info_fps_{200};
  double total_delay_{0};
  double start_time_{0};
  int process_time_ms_{0};
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_
