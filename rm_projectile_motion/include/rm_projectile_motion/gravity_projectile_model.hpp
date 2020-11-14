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
#ifndef RM_PROJECTILE_MOTION_GRAVITY_PROJECTILE_MODEL_HPP
#define RM_PROJECTILE_MOTION_GRAVITY_PROJECTILE_MODEL_HPP

#include "rm_projectile_motion/iteration_projectile_model.hpp"

namespace rm_projectile_motion {

//考虑重力的弹道重力修正工具.
class GravityProjectileModel : public IterationProjectileModel {
   public:
    GravityProjectileModel(float projectile_v);
    ~GravityProjectileModel(){};

   public:
    int forward(float given_angle, float given_x, float &h, float &t);

   private:
    float bullet_v_;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_GRAVITY_PROJECTILE_MODEL_HPP
