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