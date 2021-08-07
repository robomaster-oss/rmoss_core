// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_BASE__FIXED_PACKET_HPP_
#define RM_BASE__FIXED_PACKET_HPP_

#include <cstring>
#include <memory>

namespace rm_base
{

// 定长数据包封装
template<int capacity = 16>
class FixedPacket
{
public:
  using SharedPtr = std::shared_ptr<FixedPacket>;
  FixedPacket();
  ~FixedPacket() {}

public:
  // 设置校验位
  void pack();
  // 检查校验位，是否为合法数据帧
  int check(unsigned char * recv_buffer, int recv_len);
  // 清除缓存
  void clear();
  // copy数据到缓存buffer
  void copy_from(const unsigned char * src) {memcpy(buffer_, src, capacity);}
  // 获取缓存buffer
  const unsigned char * buffer() const {return buffer_;}

  // 自定义装载数据
  template<typename T, int data_len = sizeof(T)>
  bool load_data(T const & data, int index)
  {
    // 越界检测
    if (index > 0 && ((index + data_len) < (capacity - 1))) {
      memcpy(buffer_ + index, &data, data_len);
      return true;
    }
    return false;
  }

  // 自定义解析数据
  template<typename T, int data_len = sizeof(T)>
  bool unload_data(T & data, int index)
  {
    // 越界检测
    if (index > 0 && ((index + data_len) < (capacity - 1))) {
      memcpy(&data, buffer_ + index, data_len);
      return true;
    }
    return false;
  }

private:
  // 数据包缓存buffer
  unsigned char buffer_[capacity];
};


template<int capacity>
FixedPacket<capacity>::FixedPacket()
{
  memset(buffer_, 0, capacity);
}


template<int capacity>
void FixedPacket<capacity>::pack()
{
  buffer_[0] = 0xff;  // 帧头
  buffer_[capacity - 1] = 0x0d;  // 帧尾
}

// 数据帧检查
template<int capacity>
int FixedPacket<capacity>::check(unsigned char * tmp_buffer, int recv_len)
{
  // 检查长度
  if (recv_len != capacity) {
    return -1;
  }
  // 检查帧头，帧尾,
  if ((tmp_buffer[0] != 0xff) || (tmp_buffer[capacity - 1] != 0x0d)) {
    return -2;
  }
  return 0;
}

// 数据帧清空
template<int capacity>
void FixedPacket<capacity>::clear()
{
  memset(buffer_, 0, capacity);    // 每个字节都用0填充
}


using FixedPacket16 = FixedPacket<16>;
using FixedPacket32 = FixedPacket<32>;
using FixedPacket64 = FixedPacket<64>;

}  // namespace rm_base

#endif  // RM_BASE__FIXED_PACKET_HPP_
