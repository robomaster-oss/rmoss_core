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
#include "rm_base/fixed_packet_tool.hpp"
#include <iostream>
#include <thread>

using namespace rm_base;

FixedPacketTool::FixedPacketTool(CommDevInterface* trans_dev) {
    comm_dev_ = trans_dev;
}

FixedPacketTool::~FixedPacketTool() {}

bool FixedPacketTool::isOpen() {
    if (comm_dev_ == NULL) {
        return false;
    }
    return comm_dev_->isOpen();
}

int FixedPacketTool::sendPacket(FixedPacket packet) {
    if (isOpen()) {
        if(comm_dev_->dataSend(packet.buffer_, packet.len_)==packet.len_){
            return 0;
        }
    }
    return -1;
}

int FixedPacketTool::recvPacket(FixedPacket& packet) {
    if(!isOpen()){
        return -3;
    }
    int ret_len;
    unsigned char tmp_buffer[128];
    ret_len = comm_dev_->dataRecv(tmp_buffer, packet_recv_.len_);
    if (ret_len > 0) {
        if (packet_recv_.unPack(tmp_buffer, ret_len) == 0) {  // check packet
            packet = packet_recv_;
            return 0;
        } else {
            std::cout << "packet check error!" << std::endl;
            return -1;
        }
    } else {
        std::cout << "serial dev error" << std::endl;
        return -2;
    }
}
