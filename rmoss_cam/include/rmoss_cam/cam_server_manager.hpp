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

#ifndef RMOSS_CAM__CAM_SERVER_MANAGER_HPP_
#define RMOSS_CAM__CAM_SERVER_MANAGER_HPP_

#include <string>
#include <memory>
#include <unordered_map>

#include "rclcpp/node.hpp"
#include "rmoss_cam/cam_server.hpp"

namespace rmoss_cam
{
class CamServerManager
{
public:
  explicit CamServerManager(
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface)
  : logging_interface_(logging_interface) {}

  bool add_cam_server(std::shared_ptr<CamServer> cam_server);
  std::shared_ptr<CamServer> get_cam_server(const std::string & camera_name);

private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  std::unordered_map<std::string, std::shared_ptr<CamServer>> cam_servers_;
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__CAM_SERVER_MANAGER_HPP_
