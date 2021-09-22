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

#ifndef RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_
#define RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_

#include <memory>

#include "Eigen/Geometry"
#include "rmoss_projectile_motion/projectile_solver_interface.hpp"

namespace rmoss_projectile_motion
{

class GimbalTransformTool
{
public:
  using SharedPtr = std::shared_ptr<GimbalTransformTool>;
  explicit GimbalTransformTool(std::shared_ptr<ProjectileSolverInterface> solver = nullptr)
  : solver_(solver) {}

  void set_projectile_solver(std::shared_ptr<ProjectileSolverInterface> solver) {solver_ = solver;}
  // position :input, position of target object in gimbal frame
  // pitch,yaw : output, angle of gimbal
  bool solve(double x, double y, double z, double & pitch, double & yaw);
  bool solve(Eigen::Vector3d position, double & pitch, double & yaw)
  {
    return solve(position(0), position(1), position(2), pitch, yaw);
  }

private:
  std::shared_ptr<ProjectileSolverInterface> solver_;
};

}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__GIMBAL_TRANSFORM_TOOL_HPP_
