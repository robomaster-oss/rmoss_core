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
#include <cmath>
#include "rm_projectile_motion/gravity_projectile_model.hpp"

using namespace std;
using namespace rm_projectile_motion;

const double PI = 3.1415926535;
const float GRAVITY = 9.7913;


GravityProjectileModel::GravityProjectileModel(float projectile_v){
    bullet_v_=projectile_v;
}

//抛物线模型
// x:m,  launch_angle:rad, y:m  t:s
int GravityProjectileModel::forward(float given_angle, float given_x, float &h, float &t){
    t = given_x / (bullet_v_ * cos(given_angle));
    h = bullet_v_ * sin(given_angle) * t - GRAVITY * t * t / 2;
    return 0;
}
