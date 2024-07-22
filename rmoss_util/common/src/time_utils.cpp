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

#include "rmoss_util/time_utils.hpp"

namespace rmoss_util
{
std::chrono::steady_clock::time_point get_curr_time() {return std::chrono::steady_clock::now();}

int64_t count_time_duration(
  const std::chrono::steady_clock::time_point & begin,
  const std::chrono::steady_clock::time_point & end, TimeUnit unit)
{
  if (unit == TimeUnit::MICROSECONDS) {
    std::chrono::microseconds duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
    return duration.count();
  } else if (unit == TimeUnit::MILLISECONDS) {
    std::chrono::milliseconds duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    return duration.count();
  }
  return 0;
}

}  // namespace rmoss_util
