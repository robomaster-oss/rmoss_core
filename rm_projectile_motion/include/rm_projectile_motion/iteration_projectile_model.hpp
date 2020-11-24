/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
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
        int solve(float target_x, float target_h, float &angle);
    public:
        /////////在水平坐标系下
        //x: input,射击距离/m
        //launch_angle: input,出射角度/rad
        //y: output,射击的y落点/m
        //t: output,射击飞行时间/s
        virtual int forward(float given_angle,float given_x ,float& h, float& t) = 0;
    private:
        int iter_time_=100;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_ITERATION_PROJECTILE_MODEL_HPP
