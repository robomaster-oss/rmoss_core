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
#include "rm_projectile_motion/gaf_projectile_model.hpp"


using namespace std;
using namespace rm_projectile_motion;

const double PI = 3.1415926535;
const float GRAVITY = 9.7913;

GafProjectileModel::GafProjectileModel(float projectile_v,float projectile_k) {
    bullet_v_=projectile_v;
    bullet_model_kx_=projectile_k;
}

// air friction of x-axis is considered（仅下降阶段考虑）
void GafProjectileModel::forward(float given_angle, float given_x, float &h, float &t){
    float v=bullet_v_;
    if (given_angle > 0.01) {    //存在上升阶段
        float t0, x0, y0;  //上升阶段（最高点）
        t0 = v * sin(given_angle) / GRAVITY;
        x0 = v * cos(given_angle) * t0;
        y0 = GRAVITY * t0 * t0 / 2;
        if (given_x < x0) {  //只有上升阶段,退化成抛物线模型
            t = given_x / (v * cos(given_angle));
            h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
        } else {           //先上升,后下降
            float t1, x1;  //下降阶段
            x1 = given_x - x0;
            t1 = (exp(bullet_model_kx_ * x1) - 1) /
                 (bullet_model_kx_ * v * cos(given_angle));
            t = t0 + t1;
            h = y0 - GRAVITY * t1 * t1 / 2;
        }
    } else {
        //只有下降
        t = (exp(bullet_model_kx_ * given_x) - 1) /
            (bullet_model_kx_ * v * cos(given_angle));  //下降阶段时间
        h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
    }
}