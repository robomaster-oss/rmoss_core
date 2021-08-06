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
