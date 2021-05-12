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
#include <iostream>
#include "rm_base/comm_dev_interface.hpp"
#include "rm_base/fixed_packet.hpp"

#define RECV_BUFFER_MAX_LEN 128

namespace rm_base{
class FixedPacketTool{
    public:
        FixedPacketTool(std::shared_ptr<CommDevInterface> comm_dev):comm_dev_(comm_dev){};
        ~FixedPacketTool(){};
    public:
        bool isOpen(){
            if (comm_dev_) return comm_dev_->isOpen();
            return false;
        }
        template<int capacity>
        bool sendPacket(const FixedPacket<capacity>& packet);
        template<int capacity>
        bool recvPacket(FixedPacket<capacity> &packet);

    protected:
        std::shared_ptr<CommDevInterface> comm_dev_;
        unsigned char recv_buffer_[RECV_BUFFER_MAX_LEN];
        int recv_buf_len_;

};

template<int capacity>
bool FixedPacketTool::sendPacket(const FixedPacket<capacity>& packet)
{
    if(isOpen()) {
        if (comm_dev_->dataSend(packet.buffer(), packet.len()) == packet.len()) {
            return true;
        }
    }
    return false;
}

template<int capacity>
bool FixedPacketTool::recvPacket(FixedPacket<capacity>& packet)
{
    if (!isOpen()) {
        return false;
    }
    int recv_len;
    unsigned char tmp_buffer[RECV_BUFFER_MAX_LEN];
    int packet_len = packet.len();
    recv_len = comm_dev_->dataRecv(tmp_buffer, packet_len);
    if (recv_len > 0) {
        // check packet
        if (packet.check(tmp_buffer, recv_len) == 0) { 
            packet.copyFrom(tmp_buffer);
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
	        for(int i=0;(i+packet_len)<=recv_buf_len_;i++){
		        if(packet.check(recv_buffer_+i, packet_len) == 0){
                    packet.copyFrom(recv_buffer_+i);
                    //读取一帧后，更新接收缓存
		            int k=0;
		            for(int j=i+packet_len;j<recv_buf_len_;j++,k++){
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

}

#endif //RM_BASE_FIXED_PACKET_TOOL_HPP
