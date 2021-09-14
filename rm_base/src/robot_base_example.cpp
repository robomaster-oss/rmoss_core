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

#include "rm_base/robot_base_example.hpp"

#include <thread>
#include <memory>

namespace rm_base
{

namespace protocol_example
{
typedef enum : unsigned char
{
  GimbalAngleControl = 0x01,
  ChangeMode = 0xa1
} ProtocolExample;
}

RobotBaseExample::RobotBaseExample(
  rclcpp::Node::SharedPtr node,
  TransporterInterface::SharedPtr transporter)
: node_(node), transporter_(transporter)
{
  // init
  packet_tool_ = std::make_shared<FixedPacketTool<16>>(transporter);
  // sub
  cmd_gimbal_sub_ = node_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
    "cmd_gimbal", 10,
    std::bind(&RobotBaseExample::gimbal_cmd_cb, this, std::placeholders::_1));
  // task thread
  listen_thread_ = std::make_unique<std::thread>(&RobotBaseExample::listen_loop, this);
}

void RobotBaseExample::gimbal_cmd_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  FixedPacket<16> packet;
  packet.load_data<unsigned char>(protocol_example::GimbalAngleControl, 1);
  packet.load_data<unsigned char>(0x00, 2);
  packet.load_data<float>(msg->position.pitch, 3);
  packet.load_data<float>(msg->position.yaw, 7);
  packet_tool_->send_packet(packet);
  // delay for data send.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void RobotBaseExample::listen_loop()
{
  FixedPacket<16> packet;
  while (rclcpp::ok()) {
    if (packet_tool_->recv_packet(packet)) {
      unsigned char cmd;
      packet.unload_data(cmd, 1);
      if (cmd == (unsigned char)protocol_example::ChangeMode) {
        unsigned char mode = 0;
        packet.unload_data(mode, 2);
        if (mode == 0x00) {
          RCLCPP_INFO(node_->get_logger(), "change mode: normal mode");
        } else if (mode == 0x01) {
          RCLCPP_INFO(node_->get_logger(), "change mode: auto aim mode");
        } else {
          RCLCPP_INFO(node_->get_logger(), "change mode:  mode err!");
        }
      } else {
        // invalid cmd
      }
    }
  }
}


}  // namespace rm_base
