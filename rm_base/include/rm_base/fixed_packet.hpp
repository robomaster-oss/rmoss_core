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

#include <cstring>

namespace rm_base{

//定长数据包封装
template<int capacity>
class FixedPacket {
public:
    FixedPacket();
    ~FixedPacket(){};

public:
    //设置校验位
    void pack();
    //检查校验位，是否为合法数据帧
    int check(unsigned char* recv_buffer,int recv_len);
    //清除缓存
    void clear();
    //copy数据帧到packet缓存
    void copyFrom(const unsigned char *src){ memcpy(buffer_,src,capacity); };
    //get function
    int len() const{  return capacity; };
    const unsigned char* buffer() const{  return buffer_; };

    //自定义装载数据 
    template <typename T>
    int loadData(T const& data,int index){
        int data_len=sizeof(T);
        if(index>0&&((index+data_len)<(capacity-1))){//越界检测
            memcpy(buffer_+index, &data, data_len);
            return 0;
        }
        return 1;
    }

    //自定义解析数据
    template <typename T>
    int unloadData(T &data,int index){
        int data_len=sizeof(T);
        if(index>0&&((index+data_len)<(capacity-1))){//越界检测
            memcpy(&data,buffer_+index, data_len);
            return 0;
        } 
        return 1;
    }

private:
    //数据包缓存buffer
    unsigned char buffer_[capacity];
};


template<int capacity>
FixedPacket<capacity>::FixedPacket(){
    memset(buffer_, 0, capacity);
}


template<int capacity>
void FixedPacket<capacity>::pack(){
    buffer_[0]=0xff;//帧头
    buffer_[capacity-1]=0x0d;//帧尾
}

//数据帧检查
template<int capacity>
int FixedPacket<capacity>::check(unsigned char* tmp_buffer,int recv_len){
    //检查长度
    if(recv_len!=capacity){
        return -1;
    }
    //检查帧头，帧尾,
    if((tmp_buffer[0]!=0xff) || (tmp_buffer[capacity-1]!=0x0d)){
        return -2;
    }
    return 0;
}

//数据帧清空
template<int capacity>
void FixedPacket<capacity>::clear(){
    memset(buffer_, 0, capacity);  //每个字节都用0填充
}


using FixedPacket16=FixedPacket<16>;
using FixedPacket32=FixedPacket<32>;
using FixedPacket64=FixedPacket<64>;

}



#endif //RM_BASE_FIXED_PACKED_HPP
