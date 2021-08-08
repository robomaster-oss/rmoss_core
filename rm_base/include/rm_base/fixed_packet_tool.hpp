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

#ifndef RM_BASE__FIXED_PACKET_TOOL_HPP_
#define RM_BASE__FIXED_PACKET_TOOL_HPP_

#include <iostream>
#include <memory>
#include "rm_base/transporter_interface.hpp"
#include "rm_base/fixed_packet.hpp"

#define RECV_BUFFER_MAX_LEN 128

namespace rm_base
{

template<int capacity = 16>
class FixedPacketTool
{
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter) {}
  ~FixedPacketTool() {}

public:
  bool is_open()
  {
    if (transporter_) {
      return transporter_->is_open();
    }
    return false;
  }
  bool send_packet(const FixedPacket<capacity> & packet);
  bool recv_packet(FixedPacket<capacity> & packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  unsigned char recv_buffer_[RECV_BUFFER_MAX_LEN];
  int recv_buf_len_;
};

template<int capacity>
bool FixedPacketTool<capacity>::send_packet(const FixedPacket<capacity> & packet)
{
  if (transporter_->write(packet.buffer(), capacity) == capacity) {
    return true;
  } else {
    // reconnect
    transporter_->close();
    transporter_->open();
    return false;
  }
}

template<int capacity>
bool FixedPacketTool<capacity>::recv_packet(FixedPacket<capacity> & packet)
{
  static unsigned char tmp_buffer[RECV_BUFFER_MAX_LEN];
  int recv_len = transporter_->read(tmp_buffer, capacity);
  if (recv_len > 0) {
    // check packet
    if (packet.check(tmp_buffer, recv_len)) {
      packet.copy_from(tmp_buffer);
      return true;
    } else {
      // 如果是断帧，拼接缓存，并遍历校验，获得合法数据
      if (recv_buf_len_ + recv_len > RECV_BUFFER_MAX_LEN) {
        recv_buf_len_ = 0;
      }
      // 拼接缓存
      memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer, recv_len);
      recv_buf_len_ = recv_buf_len_ + recv_len;
      // 遍历校验
      for (int i = 0; (i + capacity) <= recv_buf_len_; i++) {
        if (packet.check(recv_buffer_ + i, capacity)) {
          packet.copy_from(recv_buffer_ + i);
          // 读取一帧后，更新接收缓存
          int k = 0;
          for (int j = i + capacity; j < recv_buf_len_; j++, k++) {
            recv_buffer_[k] = recv_buffer_[j];
          }
          recv_buf_len_ = k;
          return true;
        }
      }
      // 表明断帧，或错误帧。
      std::cout << "packet check error!" << std::endl;
      return false;
    }
  } else {
    if (transporter_) {
      // reconnect
      transporter_->close();
      transporter_->open();
    }
    std::cout << "serial dev error" << std::endl;
    return false;
  }
}

using FixedPacket16Tool = FixedPacketTool<16>;
using FixedPacket32Tool = FixedPacketTool<32>;
using FixedPacket64Tool = FixedPacketTool<64>;

}  // namespace rm_base

#endif  // RM_BASE__FIXED_PACKET_TOOL_HPP_
