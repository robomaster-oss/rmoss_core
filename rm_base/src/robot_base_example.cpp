/****************************************************************************
 *  Copyright (C) 2019 RoboMasterOS.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : robot_base_example.cpp
 *  brief : robot通信ros<->mcu封装类(例子)
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-04-02
 ***************************************************************************/
#include "robot_base/robot_base_example.h"
#include "robot_base/protocol_example.h"
#include <thread>

using namespace robot_base;

RobotBaseExample::RobotBaseExample(){

}

RobotBaseExample::~RobotBaseExample(){

}

int RobotBaseExample::init(TransDevInterface* trans_dev){
    setTransDev(trans_dev);
    //sub
    gimbal_sub_ = nh_.subscribe("gimbal_control", 10, &RobotBaseExample::gimbalCallback, this); 

}


void RobotBaseExample::gimbalCallback(const robot_msgs::GimbalInfo & info)
{
    FixedPacket packet;
    packet.loadData<unsigned char>(protocol_example::Gimbal_Angle_Control,1);
    packet.loadData<unsigned char>(0x00,2);
    packet.loadData<float>(info.pitch,3);
    packet.loadData<float>(info.yaw,7);
    packet.pack();
    sendPacket(packet);
    //delay for data send.
    std::this_thread::sleep_for( std::chrono::milliseconds(5));
}


void RobotBaseExample::listenDev(){
    FixedPacket packet;
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

        }
    }
}
