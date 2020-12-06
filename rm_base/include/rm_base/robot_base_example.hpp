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
#ifndef RM_BASE_ROBOT_BASE_EXAMPLE_HPP
#define RM_BASE_ROBOT_BASE_EXAMPLE_HPP

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "rm_base/comm_dev_interface.hpp"
#include "rm_base/fixed_packet_tool.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"

namespace rm_base{

class RobotBaseExample{
    public:
        RobotBaseExample(rclcpp::Node::SharedPtr &nh,std::shared_ptr<CommDevInterface> comm_dev);
        ~RobotBaseExample(){};
    public:
        void mcuListenThread();
    private:
        void publishTask();
        void cmdGimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
    private:
        rclcpp::Node::SharedPtr nh_;
        std::thread mcu_listen_thread_;
        //tool
        std::shared_ptr<FixedPacketTool> packet_tool_;
        //sub
        rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr cmd_gimbal_sub_;
};

}

#endif //RM_BASE_ROBOT_BASE_EXAMPLE_HPP
