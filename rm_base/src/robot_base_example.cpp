/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

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
    gimbal_ctrl_sub_ = nh_->create_subscription<rm_interfaces::msg::GimbalControl>("gimbal_control", 10, std::bind(&RobotBaseExample::gimbalCallback, this, std::placeholders::_1));
    //task thread
    mcu_listen_thread_= std::thread(&RobotBaseExample::mcuListenThread, this);
}

void RobotBaseExample::gimbalCallback(const rm_interfaces::msg::GimbalControl::SharedPtr msg)
{
    FixedPacket packet;
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
    FixedPacket packet;
    while (rclcpp::ok()){
        if (packet_tool_->recvPacket(packet) == 0){
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
