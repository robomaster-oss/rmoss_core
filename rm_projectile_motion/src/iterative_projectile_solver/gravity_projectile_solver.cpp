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

#include "rm_projectile_motion/gravity_projectile_solver.hpp"
#include <cmath>

const double GRAVITY = 9.7913;

namespace rm_projectile_motion
{

// 抛物线模型
// x:m,  launch_angle:rad, y:m  t:s
void GravityProjectileSolver::forward_motion(
  double given_angle, double given_x, double & h, double & t)
{
  t = given_x / (initial_vel_ * cos(given_angle));
  h = initial_vel_ * sin(given_angle) * t - GRAVITY * t * t / 2;
}

}  // namespace rm_projectile_motion
