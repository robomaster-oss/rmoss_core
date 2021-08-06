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
void GravityProjectileModel::forward(float given_angle, float given_x, float &h, float &t){
    t = given_x / (bullet_v_ * cos(given_angle));
    h = bullet_v_ * sin(given_angle) * t - GRAVITY * t * t / 2;
}
