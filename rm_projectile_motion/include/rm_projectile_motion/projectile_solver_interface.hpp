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
#ifndef RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_
#define RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_

namespace rm_projectile_motion
{

class ProjectileSolverInterface
{
public:
  // return true if transform successfully.
  virtual bool solve(float target_x, float target_h, float & angle) = 0;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_