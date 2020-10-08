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
#ifndef RM_BASE_ROBOT_BASE_EXAMPLE_H
#define RM_BASE_ROBOT_BASE_EXAMPLE_H

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "rm_base/comm_dev_interface.h"
#include "rm_base/fixed_packet_tool.h"
#include "rm_interfaces/msg/gimbal_control.hpp"

namespace rm_base{

class RobotBaseExample{
    public:
        RobotBaseExample(rclcpp::Node::SharedPtr &nh,CommDevInterface* trans_dev);
        ~RobotBaseExample(){};
    public:
        void mcuListenThread();
    private:
        void publishTask();
        void gimbalCallback(const rm_interfaces::msg::GimbalControl::SharedPtr msg);
    private:
        rclcpp::Node::SharedPtr nh_;
        std::thread mcu_listen_thread_;
        //tool
        std::shared_ptr<FixedPacketTool> packet_tool_;
        //sub
        rclcpp::Subscription<rm_interfaces::msg::GimbalControl>::SharedPtr gimbal_ctrl_sub_;
};

}

#endif //RM_BASE_ROBOT_BASE_EXAMPLE_H
