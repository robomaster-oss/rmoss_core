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

#ifndef RM_BASE_FIXED_PACKED_H
#define RM_BASE_FIXED_PACKED_H

#include "string.h"

#define FIXED_PACKET_MAX_LEN 64

namespace rm_base{

//定长数据包封装
class FixedPacket {
public:
    FixedPacket();
    ~FixedPacket();

public:
    int pack();
    int unPack(unsigned char* recv_buffer,int recv_len);

    template <typename T>
    int loadData(T const& data,int index);

    template <typename T>
    int unloadData(T &data,int index);

    void clear();

public:
    //数据包缓存buffer
    unsigned char buffer_[FIXED_PACKET_MAX_LEN];
    unsigned char recv_buffer_[FIXED_PACKET_MAX_LEN*2];
    int len_;
    int recv_buf_len_;
    bool flag_;//标志位。

};


/**************自定义装载数据***********************/
template <typename T>
int FixedPacket::loadData(T const& data,int index){
    int data_len=sizeof(T);
    if(index>0&&((index+data_len)<(len_-1))){//越界检测
        memcpy(buffer_+index, &data, data_len);
        return 0;
    }
    return 1;

};

/**************自定义解析数据***********************/
template <typename T>
int FixedPacket::unloadData(T &data,int index){
    int data_len=sizeof(T);
    if(index>0&&((index+data_len)<(len_-1))){//越界检测
        memcpy(&data,buffer_+index, data_len);
        return 0;
    } 
    return 1;
}; 
   

}


#endif //RM_BASE_FIXED_PACKED_H
