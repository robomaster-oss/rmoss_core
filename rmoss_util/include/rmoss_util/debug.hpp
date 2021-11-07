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

#ifndef RMOSS_UTIL__DEBUG_HPP_
#define RMOSS_UTIL__DEBUG_HPP_

#define RMOSS_DEBUG(text) \
  if (rmoss_util::get_debug()) text

namespace rmoss_util
{
// debug config
bool get_debug();
void set_debug(bool get_debug);

}  // namespace rmoss_util

#endif  // RMOSS_UTIL__DEBUG_HPP_
