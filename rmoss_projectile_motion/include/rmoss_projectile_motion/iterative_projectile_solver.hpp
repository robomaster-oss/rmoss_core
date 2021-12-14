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
#ifndef RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_SOLVER_HPP_
#define RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_SOLVER_HPP_

#include <string>

#include "rmoss_projectile_motion/projectile_solver_interface.hpp"

namespace rmoss_projectile_motion
{

// 基于数值的抛物线逆向通用迭代求解模型.
class IterativeProjectileSolver : public ProjectileSolverInterface
{
public:
  IterativeProjectileSolver()
  : max_iter_(10) {}
  void set_max_iter(int max_iter) {max_iter_ = max_iter;}
  bool solve(double target_x, double target_h, double & angle) override;
  std::string error_message() override {return error_message_;}
  ///////// 在水平坐标系下
  // x: input,射击距离/m
  // launch_angle: input,出射角度/rad
  // y: output,射击的y落点/m
  // t: output,射击飞行时间/s
  virtual void forward_motion(double given_angle, double given_x, double & h, double & t) = 0;

private:
  int max_iter_;
  std::string error_message_;
};

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_SOLVER_HPP_
