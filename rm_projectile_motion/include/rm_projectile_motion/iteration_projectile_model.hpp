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
#ifndef RM_PROJECTILE_MOTION_ITERATION_PROJECTILE_MODEL_HPP
#define RM_PROJECTILE_MOTION_ITERATION_PROJECTILE_MODEL_HPP

#include "rm_projectile_motion/projectile_model_interface.hpp"

namespace rm_projectile_motion {

//基于数值的抛物线逆向通用迭代求解模型.
class IterationProjectileModel : public ProjectileModelInterface {
    public:
        IterationProjectileModel(){};
        ~IterationProjectileModel(){};
    public:
        bool solve(float target_x, float target_h, float &angle);
    public:
        /////////在水平坐标系下
        //x: input,射击距离/m
        //launch_angle: input,出射角度/rad
        //y: output,射击的y落点/m
        //t: output,射击飞行时间/s
        virtual void forward(float given_angle,float given_x ,float& h, float& t) = 0;
    private:
        int iter_time_=100;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_ITERATION_PROJECTILE_MODEL_HPP
