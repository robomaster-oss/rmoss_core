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

#ifndef RM_BASE__UART_TRANSPORTER_HPP_
#define RM_BASE__UART_TRANSPORTER_HPP_

#include <string>
#include "rm_base/transporter_interface.hpp"

namespace rm_base
{

// 串口数据传输设备，符合通用传输接口。
class UartTransporter : public TransporterInterface
{
public:
  UartTransporter(
    std::string device_path = "/dev/ttyUSB0", int speed = 115200,
    int flow_ctrl = 0, int databits = 0, int stopbits = 1, int parity = 'N')
  : device_path_(device_path), speed_(speed), flow_ctrl_(flow_ctrl),
    databits_(databits), stopbits_(stopbits), parity_(parity) {}

  bool open() override;
  void close() override;
  bool is_open() override;
  int read(unsigned char * buffer, int len) override;
  int write(const unsigned char * buffer, int len) override;

private:
  bool set_param(
    int speed = 115200, int flow_ctrl = 0, int databits = 0,
    int stopbits = 1, int parity = 'N');

private:
  // 设备文件描述符
  int fd_{-1};
  // 设备状态
  bool is_open_{false};
  // 设备参数
  std::string device_path_;
  int speed_;
  int flow_ctrl_;
  int databits_;
  int stopbits_;
  int parity_;
};

}  // namespace rm_base

#endif  // RM_BASE__UART_TRANSPORTER_HPP_
