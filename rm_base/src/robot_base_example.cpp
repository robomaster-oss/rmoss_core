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
#include "rm_base/protocol_example.hpp"
#include <thread>

using namespace rm_base;

RobotBaseExample::RobotBaseExample(rclcpp::Node::SharedPtr &nh, std::shared_ptr<CommDevInterface> comm_dev)
{
    //init
    nh_=nh;
    packet_tool_ = std::make_shared<FixedPacketTool>(comm_dev);
    //sub
    cmd_gimbal_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>("cmd_gimbal", 10, std::bind(&RobotBaseExample::cmdGimbalCb, this, std::placeholders::_1));
    //task thread
    mcu_listen_thread_= std::thread(&RobotBaseExample::mcuListenThread, this);
}

void RobotBaseExample::cmdGimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    FixedPacket32 packet;
    packet.loadData<unsigned char>(protocol_example::Gimbal_Angle_Control, 1);
    packet.loadData<unsigned char>(0x00, 2);
    packet.loadData<float>(msg->position.pitch, 3);
    packet.loadData<float>(msg->position.yaw, 7);
    packet.pack();
    packet_tool_->sendPacket(packet);
    //delay for data send.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void RobotBaseExample::mcuListenThread()
{
    FixedPacket32 packet;
    while (rclcpp::ok()){
        if (packet_tool_->recvPacket(packet)){
            //the packet have already unpacked.
            unsigned char cmd;
            packet.unloadData(cmd, 1);
            if (cmd == (unsigned char)protocol_example::Change_Mode){
                unsigned char mode = 0;
                packet.unloadData(mode, 2);
                if (mode == 0x00){
                    RCLCPP_INFO(nh_->get_logger(), "change mode: normal mode");
                }else if (mode == 0x01){
                    RCLCPP_INFO(nh_->get_logger(), "change mode: auto aim mode");
                }else{
                    RCLCPP_INFO(nh_->get_logger(), "change mode:  mode err!");
                }
            }
            else{
                //invalid cmd
            }
        }
    }
}
