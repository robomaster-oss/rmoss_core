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

#include "rmoss_cam/cam_server_manager.hpp"

#include <memory>
#include <string>

namespace rmoss_cam
{

bool CamServerManager::add_cam_server(std::shared_ptr<CamServer> cam_server)
{
  auto camera_name = cam_server->get_camera_name();
  if (cam_servers_.find(camera_name) != cam_servers_.end()) {
    RCLCPP_ERROR(
      logging_interface_->get_logger(), "[add_cam_server] camera <%s> is already existed.",
      camera_name.c_str());
    return false;
  }
  cam_servers_[camera_name] = cam_server;
  return true;
}

std::shared_ptr<CamServer> CamServerManager::get_cam_server(const std::string & camera_name)
{
  auto item = cam_servers_.find(camera_name);
  if (item == cam_servers_.end()) {
    RCLCPP_ERROR(
      logging_interface_->get_logger(), "[get_cam_server] failed to find camera server <%s>.",
      camera_name.c_str());
    return nullptr;
  }
  return item->second;
}

}  // namespace rmoss_cam
