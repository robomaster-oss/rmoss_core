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

#include "rm_cam/virtual_cam_node.hpp"

#include <memory>
#include <string>

namespace rm_cam
{
VirtualCamNode::VirtualCamNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("virtual_cam", options);
  // declare and get parameter
  node_->declare_parameter("image_path", "");
  node_->declare_parameter("video_path", "");
  auto image_path = node_->get_parameter("image_path").as_string();
  auto video_path = node_->get_parameter("video_path").as_string();
  // create device
  std::shared_ptr<CamInterface> cam_dev;
  if (!image_path.empty()) {
    cam_dev_ = std::make_shared<rm_cam::VirtualCam>(rm_cam::VirtualCam::IMAGE_MODE, image_path);
  } else if (!video_path.empty()) {
    cam_dev_ = std::make_shared<rm_cam::VirtualCam>(rm_cam::VirtualCam::VIDEO_MODE, video_path);
  } else {
    RCLCPP_WARN(node_->get_logger(), "image_path or video_path is empty");
    return;
  }
  // create task server
  cam_server_ = std::make_shared<rm_cam::CamServer>(node_, cam_dev_);
}

}  // namespace rm_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_cam::VirtualCamNode)
