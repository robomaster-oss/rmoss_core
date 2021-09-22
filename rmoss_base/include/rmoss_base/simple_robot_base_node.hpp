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

#ifndef RMOSS_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_
#define RMOSS_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rmoss_base/transporter_interface.hpp"
#include "rmoss_base/fixed_packet_tool.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"


namespace rmoss_base
{
// Node wrapper for SimpleRobotBase
class SimpleRobotBaseNode
{
public:
  explicit SimpleRobotBaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }
  void listen_loop();

private:
  void gimbal_cmd_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<std::thread> listen_thread_;
  // tool
  TransporterInterface::SharedPtr transporter_;
  FixedPacketTool<16>::SharedPtr packet_tool_;
  // sub
  rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
};

}  // namespace rmoss_base

#endif  // RMOSS_BASE__SIMPLE_ROBOT_BASE_NODE_HPP_
