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
#include <iostream>
#include "rm_projectile_motion/iteration_projectile_model.hpp"


using namespace std;
using namespace rm_projectile_motion;

const double PI = 3.1415926535;


int IterationProjectileModel::inverse_solve(float target_x, float target_h, float &angle){
    float aimed_h, h, dh;
    float tmp_angle= 0;
    float t=0;
    int ret;
    aimed_h = target_h;
    for (int i = 0; i < iter_time_; i++) {
        tmp_angle = (float)atan2(aimed_h, target_x);
        ret = forward(tmp_angle,target_x,h,t);
        if (t > 10) {
            cout << "无法正常打到该点,阻力过大或点过高,飞行时间为："<<ret<<":"<< t << endl;
            return 2;
        }
        dh = target_h - h;
        aimed_h = aimed_h + dh;
        if (fabsf(dh) < 0.001) {
            break;
        }
    }
    if(fabsf(dh) > 0.01){
        cout << "误差距离过大："<<dh<<endl;
        return -1;
    }
    angle = tmp_angle;
    return 0; 
}

