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

#include <cstring>
#include "rm_base/fixed_packet.hpp"

using namespace rm_base;

FixedPacket::FixedPacket(int len){
    len_ = std::min(len,FIXED_PACKET_MAX_LEN);
    memset(buffer_, 0, len_);
    flag_= false;
}

FixedPacket::~FixedPacket(){

}

void FixedPacket::pack(){
    buffer_[0]=0xff;//帧头
    buffer_[len_-1]=0x0d;//帧尾
    flag_= true;
}

//数据帧检查
int FixedPacket::check(unsigned char* tmp_buffer,int recv_len){
    //检查长度
    if(recv_len!=len_){
        return -1;
    }
    //检查帧头，帧尾,
    if((tmp_buffer[0]!=0xff) || (tmp_buffer[len_-1]!=0x0d)){
        return -2;
    }
    return 0;
}

//数据帧清空
void FixedPacket::clear(){
    memset(buffer_, 0, len_);  //每个字节都用0填充
    flag_= false;
}

