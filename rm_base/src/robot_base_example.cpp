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

#include "rm_base/robot_base_example.h"
#include "rm_base/protocol_example.h"
#include <thread>

using namespace rm_base;

RobotBaseExample::RobotBaseExample():Node("test"){

}

RobotBaseExample::~RobotBaseExample(){

}

int RobotBaseExample::init(TransDevInterface* trans_dev){
    setTransDev(trans_dev);
    packet_tool_ = new FixedPacketTool(trans_dev);
    //sub
    gimbal_sub_ = nh_.subscribe("gimbal_control", 10, &RobotBaseExample::gimbalCallback, this); 
    //task thread
    listen_thread_= std::thread(&RobotBaseExample::listenThread, this);
}


void RobotBaseExample::gimbalCallback(const robot_msgs::GimbalInfo & info)
{
    FixedPacket packet;
    packet.loadData<unsigned char>(protocol_example::Gimbal_Angle_Control,1);
    packet.loadData<unsigned char>(0x00,2);
    packet.loadData<float>(info.pitch,3);
    packet.loadData<float>(info.yaw,7);
    packet.pack();
    packet_tool_->sendPacket(packet);
    //delay for data send.
    std::this_thread::sleep_for( std::chrono::milliseconds(5));
}


void RobotBaseExample::listenThread(){
    FixedPacket packet;
    while(rclcpp::ok()){
        if(recvPacket(packet)==0){
            //the packet have already unpacked.
            unsigned char cmd;
            packet.unloadData(cmd,1);
            if(cmd==(unsigned char)protocol_example::Change_Mode){
                unsigned char mode=0;
                packet.unloadData(mode,2);
                if(mode==0x00){
                ROS_INFO("change mode: normal mode");
                }else if(mode==0x01){
                ROS_INFO("change mode: auto aim mode");
                }else{
                    ROS_INFO("change mode:  mode err!");
                }
            }else{
                //invalid cmd
            }
        }
    }

}
