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

#ifndef RMOSS_CAM__CAM_NODE_FACTORY_HPP__
#define RMOSS_CAM__CAM_NODE_FACTORY_HPP__

#include <functional>
#include <memory>

#include "rclcpp_components/node_factory.hpp"
#include "rmoss_cam/cam_server_manager.hpp"

namespace rmoss_cam
{

class CamNodeFactory : public rclcpp_components::NodeFactory
{
public:
  void
  set_resource_manager(std::shared_ptr<CamServerManager> manager)
  {
    manager_ = manager;
  }
  std::shared_ptr<CamServerManager> manager_{nullptr};
};

template<typename NodeT>
class CamNodeFactoryTemplate : public CamNodeFactory
{
public:
  CamNodeFactoryTemplate() = default;
  virtual ~CamNodeFactoryTemplate() = default;

  rclcpp_components::NodeInstanceWrapper
  create_node_instance(const rclcpp::NodeOptions & options) override
  {
    auto node = std::make_shared<NodeT>(options);
    node->set_resource_manager(manager_);
    return rclcpp_components::NodeInstanceWrapper(
      node, std::bind(&NodeT::get_node_base_interface, node));
  }
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__CAM_NODE_FACTORY_HPP__
