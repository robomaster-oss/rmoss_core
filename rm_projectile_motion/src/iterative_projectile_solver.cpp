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

#include "rm_projectile_motion/iterative_projectile_solver.hpp"

#include <cmath>

namespace rm_projectile_motion
{

bool IterativeProjectileSolver::solve(float target_x, float target_h, float & angle)
{
  float aimed_h, h, dh;
  float tmp_angle = 0;
  float t = 0;
  aimed_h = target_h;
  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = static_cast<float>(atan2(aimed_h, target_x));
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      // cout << "[IterativeProjectileSolver]:当前迭代角度超过范围(-80d,80d)"<< endl;
      return false;
    }
    forward_motion(tmp_angle, target_x, h, t);
    if (t > 10) {
      // cout << "[IterativeProjectileSolver]:当前迭代飞行时间过长："<< t << endl;
      return false;
    }
    dh = target_h - h;
    aimed_h = aimed_h + dh;
    if (fabsf(dh) < 0.001) {
      break;
    }
  }
  if (fabsf(dh) > 0.01) {
    // cout << "[IterativeProjectileSolver]:误差距离过大："<<dh<<endl;
    return false;
  }
  angle = tmp_angle;
  return true;
}


}  // namespace rm_projectile_motion
