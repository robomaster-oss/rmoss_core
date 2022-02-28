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

#ifndef RMOSS_CAM__REGISTER_NODE_MACRO_HPP__
#define RMOSS_CAM__REGISTER_NODE_MACRO_HPP__

#include "class_loader/class_loader.hpp"
#include "rclcpp_components/node_factory_template.hpp"
#include "rmoss_cam/cam_node_factory.hpp"

// refer to https://github.com/ros2/rclcpp/rclcpp_components.
#define RMOSS_CAM_COMPONENTS_REGISTER_NODE(NodeClass) \
  CLASS_LOADER_REGISTER_CLASS( \
    rclcpp_components::NodeFactoryTemplate<NodeClass>, \
    rclcpp_components::NodeFactory) \
  CLASS_LOADER_REGISTER_CLASS( \
    rmoss_cam::CamNodeFactoryTemplate<NodeClass>, \
    rmoss_cam::CamNodeFactory)

#endif  // RMOSS_CAM__REGISTER_NODE_MACRO_HPP__
