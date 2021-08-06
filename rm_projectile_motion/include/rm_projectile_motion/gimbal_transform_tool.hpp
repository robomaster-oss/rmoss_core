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
#ifndef RM_PROJECTILE_MOTION_GIMBAL_TRANSFORM_TOOL_HPP
#define RM_PROJECTILE_MOTION_GIMBAL_TRANSFORM_TOOL_HPP

#include "rm_projectile_motion/projectile_model_interface.hpp"
#include <opencv2/opencv.hpp>

namespace rm_projectile_motion {

class GimbalTransformTool {
    public:
        GimbalTransformTool(){};
        ~GimbalTransformTool(){};
    public:
        void setProjectileModel(std::shared_ptr<ProjectileModelInterface> model);
    public:
        // position :input, position of target object in gimbal frame
        // pitch,yaw : output, angle of gimbal
        bool transform(cv::Point3f position, float &pitch, float &yaw);
        //bool transform(cv::Point3f position,float current_pitch,float &delta_pitch, float &delta_yaw);
    private:
        std::shared_ptr<ProjectileModelInterface> model_;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_GIMBAL_TRANSFORM_TOOL_HPP
