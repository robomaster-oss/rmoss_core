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

#ifndef RMOSS_BASE__FIXED_PACKET_HPP_
#define RMOSS_BASE__FIXED_PACKET_HPP_

#include <cstring>
#include <memory>

namespace rmoss_base
{

// 定长数据包封装
// [head_byte(0xff),...(data_bytes)...,check_byte,tail_byte(0x0d)]
template<int capacity = 16>
class FixedPacket
{
public:
  using SharedPtr = std::shared_ptr<FixedPacket>;
  FixedPacket()
  {
    memset(buffer_, 0, capacity);
    buffer_[0] = 0xff;  // 帧头
    buffer_[capacity - 1] = 0x0d;  // 帧尾
  }

public:
  /**
   * @brief Flush buffer
   * 清除缓存, date_bytes和check_byte都用0填充
   */
  void clear() {memset(buffer_ + 1, 0, capacity - 2);}
  /**
   * @brief Set the check byte
   * 设置flag
   * @param check_byte
   */
  void set_check_byte(uint8_t check_byte) {buffer_[capacity - 2] = check_byte;}
  /**
   * @brief Copy data to buffer
   * copy数据到缓存buffer
   * @param src
   */
  void copy_from(const void * src) {memcpy(buffer_, src, capacity);}
  /**
   * @brief Get buffer
   * 获取缓存buffer
   * @return const uint8_t*
   */
  const uint8_t * buffer() const {return buffer_;}

  /**
   * @brief Self-define data loader
   * 自定义装载数据
   * @tparam T: Data type
   * @tparam data_len: Data length
   * @param data
   * @param index
   * @return true
   * @return false
   */
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

  /**
   * @brief Self-define data reader
   * 自定义解析数据
   * @tparam T
   * @tparam data_len
   * @param data
   * @param index
   * @return true
   * @return false
   */
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
  uint8_t buffer_[capacity];  // NOLINT
};

using FixedPacket16 = FixedPacket<16>;
using FixedPacket32 = FixedPacket<32>;
using FixedPacket64 = FixedPacket<64>;

}  // namespace rmoss_base

#endif  // RMOSS_BASE__FIXED_PACKET_HPP_
