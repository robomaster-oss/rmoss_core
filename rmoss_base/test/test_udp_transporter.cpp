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

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "rmoss_base/udp_transporter.hpp"

TEST(UdpTransporter, open)
{
  auto transporter1 = std::make_shared<rmoss_base::UdpTransporter>(10008, 10009);
  EXPECT_FALSE(transporter1->is_open());
  auto ret1 = transporter1->open();
  ASSERT_TRUE(ret1);
  EXPECT_TRUE(transporter1->is_open());
  transporter1->close();
}

TEST(UdpTransporter, send_and_recv)
{
  auto transporter1 = std::make_shared<rmoss_base::UdpTransporter>(10008, 10009);
  auto transporter2 = std::make_shared<rmoss_base::UdpTransporter>(10009, 10008);
  auto ret1 = transporter1->open();
  ASSERT_TRUE(ret1);
  auto ret2 = transporter2->open();
  ASSERT_TRUE(ret2);
  // send buffer and recv buffer2
  uint8_t buffer[32], buffer2[32];
  memset(buffer, 0, 32);
  memset(buffer2, 0, 32);
  for (int i = 0; i < 32; i++) {
    buffer[i] = i;
  }
  transporter1->write(buffer, 32);
  transporter2->read(buffer2, 32);
  for (int i = 0; i < 32; i++) {
    EXPECT_EQ(buffer2[i], buffer[i]);
  }
  transporter1->close();
  transporter2->close();
}
