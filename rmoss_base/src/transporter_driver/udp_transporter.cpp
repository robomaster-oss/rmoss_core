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

#include "rmoss_base/udp_transporter.hpp"

#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <memory>
#include <string>

namespace rmoss_base
{

bool UdpTransporter::open()
{
  if (is_open_) {
    return true;
  }
  // recv port
  sock_recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_recv_fd_ < 0) {
    error_message_ = "create socket failed";
    return false;
  }
  struct sockaddr_in addr_serv;
  memset(&addr_serv, 0, sizeof(addr_serv));
  addr_serv.sin_family = AF_INET;  // 使用IPV4地址
  addr_serv.sin_port = htons(port_);  // 设备自身端口
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  // 绑定socket
  if (bind(sock_recv_fd_, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) {
    error_message_ = "bind port " + std::to_string(port_) + " failed";
    return false;
  }
  // set send port
  sock_send_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&to_addr_, 0, sizeof(to_addr_));
  to_addr_.sin_family = AF_INET;
  to_addr_.sin_port = htons(target_port_);
  to_addr_.sin_addr.s_addr = inet_addr(target_ip_.c_str());
  to_addr_len_ = sizeof(to_addr_);
  // success
  is_open_ = true;
  return true;
}

void UdpTransporter::close()
{
  if (is_open_) {
    ::close(sock_recv_fd_);
    ::close(sock_send_fd_);
    is_open_ = false;
  }
}

bool UdpTransporter::is_open()
{
  return is_open_;
}

int UdpTransporter::read(void * buffer, size_t len)
{
  return recvfrom(sock_recv_fd_, buffer, len, 0, NULL, NULL);
}

int UdpTransporter::write(const void * buffer, size_t len)
{
  return sendto(sock_send_fd_, buffer, len, 0, (struct sockaddr *)&to_addr_, to_addr_len_);
}


}  // namespace rmoss_base
