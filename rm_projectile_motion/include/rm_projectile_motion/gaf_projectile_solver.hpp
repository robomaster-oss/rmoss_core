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

#ifndef RM_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_
#define RM_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_

#include "rm_projectile_motion/iterative_projectile_solver.hpp"

namespace rm_projectile_motion
{

// 考虑重力和空气阻力（gaf:Gravity and Air Frication）的弹道重力修正工具.
class GafProjectileSolver : public IterativeProjectileSolver
{
public:
  GafProjectileSolver(float initial_vel, float friction_coeff);
  ~GafProjectileSolver() = default;

public:
  void forward_motion(float given_angle, float given_x, float & h, float & t);

private:
  // parameter
  float initial_vel_;
  float friction_coeff_;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_
