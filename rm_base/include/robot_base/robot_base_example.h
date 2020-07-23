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
 *  file  : robot_base_example.h
 *  brief : robot通信ros<->mcu封装顶层模块(例子)
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-1-02
 ***************************************************************************/
#ifndef ROBOT_BASE_ROBOT_BASE_EXAMPLE_H
#define ROBOT_BASE_ROBOT_BASE_EXAMPLE_H

#include "ros/ros.h"
#include "robot_base/fixed_common_base.h"
#include "robot_msgs/GimbalInfo.h"

namespace robot_base{


class RobotBaseExample : public FixedCommonBase {
    public:
        RobotBaseExample();
        ~RobotBaseExample();
    public:
        int init(TransDevInterface* trans_dev);
        void listenDev();
    private:
        //ros msg topic recieve
        void gimbalCallback(const robot_msgs::GimbalInfo & info);
    private:
        ros::NodeHandle nh_;
        //sub
        ros::Subscriber gimbal_sub_;
};

}

#endif //ROBOT_BASE_ROBOT_BASE_EXAMPLE_H
