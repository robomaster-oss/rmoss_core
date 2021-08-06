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

#include <opencv2/opencv.hpp>
#include "rm_projectile_motion/gimbal_transform_tool.hpp"

using namespace rm_projectile_motion;
using namespace std;


void GimbalTransformTool::setProjectileModel(std::shared_ptr<ProjectileModelInterface> model){
    model_ = model;
}

bool GimbalTransformTool::transform(cv::Point3f position, float &pitch, float &yaw){
    if(!model_){
        //if model is NULL,use line model.
        pitch = -(float)(atan2(position.z, position.x));
    }else{
        float angle;
        if(model_->solve( position.z,position.x,angle)){
            pitch = -angle; 
        }else{
            return false;
        }
    }
    yaw = (float)(atan2(position.y, position.x));
    return true;
}