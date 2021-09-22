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

#include "dummy_transporter.hpp"
#include "rmoss_base/fixed_packet.hpp"

TEST(FixedPacket, load_data_and_unload_data)
{
  rmoss_base::FixedPacket<32> packet1;
  packet1.clear();
  // int数据加载和卸载
  int a = 1, b = 0;
  packet1.load_data(a, 1);
  packet1.unload_data<int>(b, 1);
  EXPECT_EQ(a, b);
  // float数据加载和卸载
  float a2 = 1.11, b2 = 0;
  packet1.load_data(a2, 5);
  packet1.unload_data<float>(b2, 5);
  EXPECT_EQ(a2, b2);
}

TEST(FixedPacket, bounded_check)
{
  rmoss_base::FixedPacket<32> packet1;
  packet1.clear();
  int a = 1;
  // 越界检查
  bool ret1 = packet1.load_data(a, 32);
  EXPECT_FALSE(ret1);
  bool ret2 = packet1.load_data(a, -1);
  EXPECT_FALSE(ret2);
}

TEST(FixedPacket, copy_from)
{
  rmoss_base::FixedPacket<32> packet1;
  packet1.clear();
  unsigned char buffer[32];
  memset(buffer, 0, 32);
  int a = 4, b = 0;
  memcpy(buffer + 3, &a, 4);
  // copy_from
  packet1.copy_from(buffer);
  packet1.unload_data(b, 3);
  EXPECT_EQ(a, b);
}
