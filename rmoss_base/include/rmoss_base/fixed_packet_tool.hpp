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

#ifndef RMOSS_BASE__FIXED_PACKET_TOOL_HPP_
#define RMOSS_BASE__FIXED_PACKET_TOOL_HPP_

#include <iostream>
#include <memory>
#include <stdexcept>
#include <queue>
#include <thread>
#include <mutex>

#include "rmoss_base/transporter_interface.hpp"
#include "rmoss_base/fixed_packet.hpp"

namespace rmoss_base
{
template<int capacity = 16>
class FixedPacketTool
{
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  FixedPacketTool() = delete;
  /**
   * @brief Construct a new Fixed Packet Tool object
   *
   * @param transporter transport interface
   */
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter)
  {
    if (!transporter) {
      throw std::invalid_argument("transporter is nullptr");
    }
  }

  ~FixedPacketTool() {enable_realtime_send(false);}

  /**
   * @brief Test interface is open or not
   *
   * @return true
   * @return false
   */
  bool is_open() {return transporter_->is_open();}
  /**
   * @brief enable realtime send
   *
   * @param enable
   */
  void enable_realtime_send(bool enable);
  /**
   * @brief Send a packet
   *
   * @param packet
   * @return true
   * @return false
   */
  bool send_packet(const FixedPacket<capacity> & packet);
  /**
   * @brief Recieve a packet
   *
   * @param packet
   * @return true
   * @return false
   */
  bool recv_packet(FixedPacket<capacity> & packet);

private:
  bool check_packet(uint8_t * tmp_buffer, int recv_len);
  bool simple_send_packet(const FixedPacket<capacity> & packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  // data
  uint8_t tmp_buffer_[capacity];  // NOLINT
  uint8_t recv_buffer_[capacity * 2];  // NOLINT
  int recv_buf_len_;
  // for realtime sending
  bool use_realtime_send_{false};
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

template<int capacity>
bool FixedPacketTool<capacity>::check_packet(uint8_t * buffer, int recv_len)
{
  // 检查长度
  if (recv_len != capacity) {
    return false;
  }
  // 检查帧头，帧尾,
  if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d)) {
    return false;
  }
  // TODO(gezp): 检查check_byte(buffer[capacity-2]),可采用异或校验(BCC)
  return true;
}

template<int capacity>
bool FixedPacketTool<capacity>::simple_send_packet(const FixedPacket<capacity> & packet)
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
void FixedPacketTool<capacity>::enable_realtime_send(bool enable)
{
  if (enable == use_realtime_send_) {
    return;
  }
  if (enable) {
    use_realtime_send_ = true;
    realtime_send_thread_ = std::make_unique<std::thread>(
      [&]() {
        FixedPacket<capacity> packet;
        while (use_realtime_send_) {
          bool empty = true;
          {
            std::lock_guard<std::mutex> lock(realtime_send_mut_);
            empty = realtime_packets_.empty();
            if (!empty) {
              packet = realtime_packets_.front();
              realtime_packets_.pop();
            }
          }
          if (!empty) {
            simple_send_packet(packet);
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
      });
  } else {
    use_realtime_send_ = false;
    realtime_send_thread_->join();
    realtime_send_thread_.reset();
  }
}

template<int capacity>
bool FixedPacketTool<capacity>::send_packet(const FixedPacket<capacity> & packet)
{
  if (use_realtime_send_) {
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(packet);
    return true;
  } else {
    return simple_send_packet(packet);
  }
}

template<int capacity>
bool FixedPacketTool<capacity>::recv_packet(FixedPacket<capacity> & packet)
{
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  if (recv_len > 0) {
    // check packet
    if (check_packet(tmp_buffer_, recv_len)) {
      packet.copy_from(tmp_buffer_);
      return true;
    } else {
      // 如果是断帧，拼接缓存，并遍历校验，获得合法数据
      if (recv_buf_len_ + recv_len > capacity * 2) {
        recv_buf_len_ = 0;
      }
      // 拼接缓存
      memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
      recv_buf_len_ = recv_buf_len_ + recv_len;
      // 遍历校验
      for (int i = 0; (i + capacity) <= recv_buf_len_; i++) {
        if (check_packet(recv_buffer_ + i, capacity)) {
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
      return false;
    }
  } else {
    // reconnect
    transporter_->close();
    transporter_->open();
    // 串口错误
    return false;
  }
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace rmoss_base

#endif  // RMOSS_BASE__FIXED_PACKET_TOOL_HPP_
