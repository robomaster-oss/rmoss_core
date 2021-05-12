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
#include <cstring>



using namespace rm_base;

FixedPacketTool::FixedPacketTool(std::shared_ptr<CommDevInterface> comm_dev)
{
    comm_dev_ = comm_dev;
}

FixedPacketTool::~FixedPacketTool() { }

bool FixedPacketTool::isOpen()
{
    if (comm_dev_) {
        return comm_dev_->isOpen();
    }
    return false;
}

bool FixedPacketTool::sendPacket(FixedPacket packet)
{
    if (isOpen()) {
        if (comm_dev_->dataSend(packet.buffer_, packet.len_) == packet.len_) {
            return true;
        }
    }
    return false;
}

bool FixedPacketTool::recvPacket(FixedPacket& packet)
{
    if (!isOpen()) {
        return false;
    }
    int recv_len;
    unsigned char tmp_buffer[FIXED_PACKET_MAX_LEN];
    recv_len = comm_dev_->dataRecv(tmp_buffer, packet.len_);
    if (recv_len > 0) {
        if (packet.check(tmp_buffer, recv_len) == 0) { // check packet
            memcpy(packet.buffer_,tmp_buffer,packet.len_);
            return true;
        } else {
            //如果是断帧，拼接缓存，并遍历校验，获得合法数据
            if(recv_buf_len_+recv_len>RECV_BUFFER_MAX_LEN){
	            recv_buf_len_=0;
            }
            //拼接缓存
            memcpy(recv_buffer_+recv_buf_len_,tmp_buffer, recv_len);
            recv_buf_len_=recv_buf_len_+recv_len;
            //遍历校验
	        for(int i=0;(i+packet.len_)<=recv_buf_len_;i++){
		        if(packet.check(recv_buffer_+i, packet.len_) == 0){
                    memcpy(packet.buffer_,recv_buffer_+i,packet.len_);
                    //读取一帧后，更新接收缓存
		            int k=0;
		            for(int j=i+packet.len_;j<recv_buf_len_;j++,k++){
			            recv_buffer_[k]=recv_buffer_[j];
		            }
		            recv_buf_len_=k; 
                    return true;                   
  		        }
            }
            //表明断帧，或错误帧。
            std::cout << "packet check error!" << std::endl;
            return false;
        }
    } else {
        std::cout << "serial dev error" << std::endl;
        return false;
    }
}
