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

#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"

namespace rmoss_projectile_motion
{

bool GimbalTransformTool::solve(double x, double y, double z, double & pitch, double & yaw)
{
  if (!solver_) {
    // if model is nullptr, use line model.
    pitch = -atan2(z, x);
  } else {
    double angle;
    if (solver_->solve(x, z, angle)) {
      pitch = -angle;
    } else {
      error_message_ = solver_->error_message();
      return false;
    }
  }
  yaw = atan2(y, x);
  return true;
}

}  // namespace rmoss_projectile_motion
