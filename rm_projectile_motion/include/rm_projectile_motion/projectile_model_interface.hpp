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
#ifndef RM_PROJECTILE_MOTION_PROJECTILE_MODEL_INTERFACE_HPP
#define RM_PROJECTILE_MOTION_PROJECTILE_MODEL_INTERFACE_HPP

namespace rm_projectile_motion {

class ProjectileModelInterface {
   public:
    // return: state, 0 if transform successfully.
    virtual int solve(float target_x,float target_h, float &angle) = 0;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_PROJECTILE_MODEL_INTERFACE_HPP
