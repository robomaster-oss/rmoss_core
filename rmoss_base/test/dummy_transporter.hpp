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

#ifndef DUMMY_TRANSPORTER_HPP_
#define DUMMY_TRANSPORTER_HPP_

#include <unistd.h>

#include <memory>
#include <string>

#include "rmoss_base/transporter_interface.hpp"

// FIFO传输设备，用于测试。
class FifoTransporter : public rmoss_base::TransporterInterface
{
public:
  FifoTransporter(int fifo_rd_fd, int fifo_wr_fd)
  : fifo_rd_fd_(fifo_rd_fd), fifo_wr_fd_(fifo_wr_fd)
  {}

  bool open() override
  {
    return true;
  }
  void close() override
  {
  }
  bool is_open() override
  {
    return true;
  }
  int read(void * buffer, size_t len) override
  {
    return ::read(fifo_rd_fd_, buffer, len);
  }
  int write(const void * buffer, size_t len) override
  {
    return ::write(fifo_wr_fd_, buffer, len);
  }
  std::string error_message() override
  {
    return error_message_;
  }

private:
  int fifo_rd_fd_;
  int fifo_wr_fd_;
  std::string error_message_;
};

class TransporterFactory
{
public:
  TransporterFactory()
  {
    pipe(fds1);
    pipe(fds2);
    transporter1_ = std::make_shared<FifoTransporter>(fds1[0], fds2[1]);
    transporter2_ = std::make_shared<FifoTransporter>(fds2[0], fds1[1]);
  }
  rmoss_base::TransporterInterface::SharedPtr get_transporter1()
  {
    return transporter1_;
  }
  rmoss_base::TransporterInterface::SharedPtr get_transporter2()
  {
    return transporter2_;
  }

private:
  int fds1[2];
  int fds2[2];
  rmoss_base::TransporterInterface::SharedPtr transporter1_;
  rmoss_base::TransporterInterface::SharedPtr transporter2_;
};

#endif  // DUMMY_TRANSPORTER_HPP_
