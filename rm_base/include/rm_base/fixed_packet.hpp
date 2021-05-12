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

#ifndef RM_BASE_FIXED_PACKED_HPP
#define RM_BASE_FIXED_PACKED_HPP

#include <string>


#define FIXED_PACKET_MAX_LEN 64

namespace rm_base{

//定长数据包封装
class FixedPacket {
public:
    FixedPacket(int len=32);
    ~FixedPacket();

public:
    void pack();
    int check(unsigned char* recv_buffer,int recv_len);
    void clear();

    template <typename T>
    int loadData(T const& data,int index);

    template <typename T>
    int unloadData(T &data,int index);

public:
    //数据包缓存buffer
    unsigned char buffer_[FIXED_PACKET_MAX_LEN];
    int len_;
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
}

/**************自定义解析数据***********************/
template <typename T>
int FixedPacket::unloadData(T &data,int index){
    int data_len=sizeof(T);
    if(index>0&&((index+data_len)<(len_-1))){//越界检测
        memcpy(&data,buffer_+index, data_len);
        return 0;
    } 
    return 1;
}
   

}


#endif //RM_BASE_FIXED_PACKED_HPP
