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

#include "rmoss_projectile_motion/iterative_projectile_tool.hpp"

#include <cmath>

namespace rmoss_projectile_motion
{

bool IterativeProjectileTool::solve(double target_x, double target_h, double & angle)
{
  double aimed_h, h;
  double dh = 0;
  double tmp_angle = 0;
  double t = 0;
  aimed_h = target_h;
  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = atan2(aimed_h, target_x);
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      error_message_ = "iterative angle is out of range(-80d,80d)";
      return false;
    }
    forward_motion_func_(tmp_angle, target_x, h, t);
    if (t > 10) {
      error_message_ = "motion time(" + std::to_string(t) + ") is too long";
      return false;
    }
    dh = target_h - h;
    aimed_h = aimed_h + dh;
    if (fabs(dh) < 0.001) {
      break;
    }
  }
  if (fabs(dh) > 0.01) {
    error_message_ = "height error(" + std::to_string(dh) + ") is too large";
    return false;
  }
  angle = tmp_angle;
  return true;
}

}  // namespace rmoss_projectile_motion
