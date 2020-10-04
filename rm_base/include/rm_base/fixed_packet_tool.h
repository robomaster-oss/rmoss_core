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

#ifndef RM_BASE_FIXED_PACKET_TOOL_H
#define RM_BASE_FIXED_PACKET_TOOL_H

#include <string>
#include "rm_base/comm_dev_interface.h"
#include "rm_base/fixed_packet.h"


namespace rm_base{
class FixedPacketTool{
    public:
        FixedPacketTool(CommDevInterface* comm_dev);
        ~FixedPacketTool();
    public:
        bool isOpen();
        int sendPacket(FixedPacket packet);
        int recvPacket(FixedPacket &packet);

    protected:
        CommDevInterface* comm_dev_;
        unsigned char recv_buffer_[128];
        FixedPacket packet_recv_;


};


}

#endif //RM_BASE_FIXED_PACKET_TOOL_H
