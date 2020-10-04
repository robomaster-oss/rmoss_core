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
#include "rm_base/fixed_packet.h"

using namespace rm_base;

FixedPacket::FixedPacket(){
    len_=32;
    recv_buf_len_=0;
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


//数据帧解析，包含数据帧检查
int FixedPacket::unPack(unsigned char* tmp_buffer,int recv_len){
    //检查帧头，帧尾,长度,如果正常帧，直接unpack
    if(recv_len==len_&&tmp_buffer[0]==0xff&&tmp_buffer[len_-1]==0x0d){
        memcpy(buffer_,tmp_buffer,len_);
        recv_buf_len_=0;
        return 0;
    }
    //如果是断帧，缓存待拼接
    if(recv_buf_len_+recv_len>FIXED_PACKET_MAX_LEN*2){//如果超过缓存，直接清空
	    recv_buf_len_=0;
    }
    memcpy(recv_buffer_+recv_buf_len_,tmp_buffer, recv_len);//缓存
    recv_buf_len_=recv_buf_len_+recv_len;
    //遍历校验
	for(int i=0;(i+len_)<=recv_buf_len_;i++){
		if(recv_buffer_[i]==0xff&&recv_buffer_[i+len_-1]==0x0d){
		    memcpy(buffer_,recv_buffer_+i,len_);
		    int k=0;
		    for(int j=i+len_;j<recv_buf_len_;j++){//读取一帧后，更新缓存
			    recv_buffer_[k]=recv_buffer_[j];
                k++;
		    }
		    recv_buf_len_=k; 
            return 0;                   
  		}
    }
    //如果一直校验不成功，当recv_len>2*len，清空缓存
    if(recv_buf_len_>=len_*2){
		recv_buf_len_=0;
        return -1;
    }
    return -2;//表明断帧，或错误帧。

}
//数据帧清空
void FixedPacket::clear(){
    memset(buffer_, 0, len_);  //每个字节都用0填充
    flag_= false;
}

