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
 *  file  : fixed_common_base.h
 *  brief : robot通信ros<->mcu封装类
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-1-02
 ***************************************************************************/
#ifndef ROBOT_BASE_FIXED_COMMON_BASE_H
#define ROBOT_BASE_FIXED_COMMON_BASE_H

#include <string>
#include "robot_base/trans_dev_interface.h"
#include "robot_base/fixed_packet.h"


namespace robot_base{


class FixedCommonBase{
    public:
        FixedCommonBase();
        ~FixedCommonBase();
        
    public:
        void setTransDev(TransDevInterface* trans_dev);
        bool isOpen();
        int sendPacket(FixedPacket packet);
        int recvPacket(FixedPacket &packet);

    protected:
        TransDevInterface* trans_dev_;
        unsigned char recv_buffer_[128];
        FixedPacket packet_recv_;


};


}

#endif //ROBOT_BASE_FIXED_COMMON_BASE_H
