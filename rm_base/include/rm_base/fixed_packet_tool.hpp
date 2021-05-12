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

#ifndef RM_BASE_FIXED_PACKET_TOOL_HPP
#define RM_BASE_FIXED_PACKET_TOOL_HPP

#include <memory>
#include "rm_base/comm_dev_interface.hpp"
#include "rm_base/fixed_packet.hpp"

#define RECV_BUFFER_MAX_LEN 128

namespace rm_base{
class FixedPacketTool{
    public:
        FixedPacketTool(std::shared_ptr<CommDevInterface> comm_dev);
        ~FixedPacketTool();
    public:
        bool isOpen();
        bool sendPacket(const FixedPacket& packet);
        bool recvPacket(FixedPacket &packet);

    protected:
        std::shared_ptr<CommDevInterface> comm_dev_;
        unsigned char recv_buffer_[RECV_BUFFER_MAX_LEN];
        int recv_buf_len_;

};


}

#endif //RM_BASE_FIXED_PACKET_TOOL_HPP
