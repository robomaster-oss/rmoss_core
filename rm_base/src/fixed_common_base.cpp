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
 *  file  : fixed_common_base.cpp
 *  brief : robot通信ros<->mcu封装类
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-04-01
 ***************************************************************************/
#include "robot_base/fixed_common_base.h"
#include <iostream>
#include <thread>

using namespace robot_base;

FixedCommonBase::FixedCommonBase() {
    trans_dev_ = NULL;
}

FixedCommonBase::~FixedCommonBase() {}

void FixedCommonBase::setTransDev(TransDevInterface* trans_dev) {
    trans_dev_ = trans_dev;
}

bool FixedCommonBase::isOpen() {
    if (trans_dev_ == NULL) {
        return false;
    }
    return trans_dev_->isOpen();
}

int FixedCommonBase::sendPacket(FixedPacket packet) {
    if (isOpen()) {
        if(trans_dev_->dataSend(packet.buffer_, packet.len_)==packet.len_){
            return 0;
        }
    }
    return -1;
}

int FixedCommonBase::recvPacket(FixedPacket& packet) {
    if(!isOpen()){
        return -3;
    }
    int ret_len;
    unsigned char tmp_buffer[128];
    ret_len = trans_dev_->dataRecv(tmp_buffer, packet_recv_.len_);
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
