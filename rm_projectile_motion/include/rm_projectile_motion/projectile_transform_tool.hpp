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
#ifndef RM_PROJECTILE_MOTION_PROJECTILE_TRANSFORM_TOOL_HPP
#define RM_PROJECTILE_MOTION_PROJECTILE_TRANSFORM_TOOL_HPP

#include "rm_projectile_motion/projectile_model_interface.hpp"
#include <opencv2/opencv.hpp>

namespace rm_projectile_motion {

class ProjectileTransformTool {
    public:
        ProjectileTransformTool(){};
        ~ProjectileTransformTool(){};
    public:
        void setModel(ProjectileModelInterface *model);
    public:
        // position :input, position of target object in gimbal frame (向前为x方向)
        // pitch,yaw : output, angle of gimbal
        // return: state, 0 if transform successfully.
        int transform(cv::Point3f position, float &pitch, float &yaw);
    private:
        ProjectileModelInterface *model_;
};

}  // namespace rm_projectile_motion

#endif  // RM_PROJECTILE_MOTION_PROJECTILE_TRANSFORM_TOOL_HPP
