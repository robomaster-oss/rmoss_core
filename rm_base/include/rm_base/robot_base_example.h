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
#define RM_BASE_ROBOT_BASE_EXAMPLE_H
#define RM_BASE_ROBOT_BASE_EXAMPLE_H

#include <rclcpp/rclcpp.hpp>
#include "rm_base/fixed_packet_tool.h"
#include "rm_interfaces/msg/GimbalControl.h"

namespace rm_base{

class RobotBaseExample : public rclcpp::Node {
    public:
        RobotBaseExample();
        ~RobotBaseExample();
    public:
        int init(TransDevInterface* trans_dev);
        void listenThread();
    private:
        //ros msg topic recieve
        void gimbalCallback(const robot_msgs::GimbalInfo & info);
    private:
        ros::NodeHandle nh_;
        std::thread listen_thread_;
        //tool
        FixedPacketTool packet_tool_;
        //sub
        ros::Subscriber gimbal_sub_;
        //
};

}

#endif //ROBOT_BASE_ROBOT_BASE_EXAMPLE_H
