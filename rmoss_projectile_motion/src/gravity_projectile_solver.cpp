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

#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"

#include <cmath>
#include <string>
#include <memory>

const double GRAVITY = 9.7913;

namespace rmoss_projectile_motion
{

GravityProjectileSolver::GravityProjectileSolver(double initial_vel)
: initial_vel_(initial_vel)
{
  // forward motion model: gravity model (抛物型模型)
  auto forward_motion = [&](double given_angle, double given_x, double & h, double & t) {
      t = given_x / (initial_vel_ * cos(given_angle));
      h = initial_vel_ * sin(given_angle) * t - GRAVITY * t * t / 2;
    };
  // configure iterative tool
  iterative_tool_ = std::make_shared<IterativeProjectileTool>();
  iterative_tool_->set_forward_motion(forward_motion);
  iterative_tool_->set_max_iter(20);
}

bool GravityProjectileSolver::solve(double target_x, double target_h, double & angle)
{
  return iterative_tool_->solve(target_x, target_h, angle);
}

std::string GravityProjectileSolver::error_message()
{
  return iterative_tool_->error_message();
}

}  // namespace rmoss_projectile_motion
