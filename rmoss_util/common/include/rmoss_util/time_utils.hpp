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

#ifndef RMOSS_UTIL__TIME_UTILS_HPP_
#define RMOSS_UTIL__TIME_UTILS_HPP_

#include <chrono>

// author: wyx
// email : 1418555317@qq.com

namespace rmoss_util
{
enum class TimeUnit
{
  MICROSECONDS,  // us
  MILLISECONDS,  // ms
};
// 时间工具
/** 获取当前时间
* @return: std::chrono::steady_clock::time_point，可用auto接收
*/
std::chrono::steady_clock::time_point get_curr_time();

/** 重载计时函数
* @param: begin, 计时的开始时间
* @param: end, 计时的结束时间
*/
int64_t count_time_duration(
  const std::chrono::steady_clock::time_point & begin,
  const std::chrono::steady_clock::time_point & end, TimeUnit unit = TimeUnit::MILLISECONDS);

}  // namespace rmoss_util

#endif  // RMOSS_UTIL__TIME_UTILS_HPP_
