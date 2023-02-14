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
#ifndef RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_
#define RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_

#include <string>
#include <functional>

namespace rmoss_projectile_motion
{

// 基于数值的通用抛物线逆运动学迭代求解工具
class IterativeProjectileTool
{
  // 在水平坐标系下
  // given_x: input,射击距离/m
  // given_angle: input,出射角度/rad
  // h: output,射击的落点高度/m
  // t: output,射击飞行时间/s
  typedef std::function<void (double given_angle, double given_x, double & h,
      double & t)> ForwardMotionFunc;

public:
  IterativeProjectileTool() {}
  /**
   * @brief Set the max iteration number
   *
   * @param max_iter max iteration number
   */
  void set_max_iter(int max_iter) {max_iter_ = max_iter;}
  /**
   * @brief Set the forward motion fuction for the tool
   * 设置弹道前向运动解算函数
   * 函数的类型为: void f(ouble given_angle, double given_x, double & h, double & t)
   * 在水平坐标系下
   * given_x: input,射击距离/m
   * given_angle: input,出射角度/rad
   * h: output,射击的落点高度/m
   * t: output,射击飞行时间/s
   * @param forward_motion forward_motion function for the tool
   */
  void set_forward_motion(ForwardMotionFunc forward_motion) {forward_motion_func_ = forward_motion;}
  /**
   * @brief Solve projectile motion
   *
   * @param target_x horizontal distance of target (m)
   * @param target_h vertical distance of target (m)
   * @param angle ouput angle (rad)
   * @return true
   * @return false
   */
  bool solve(double target_x, double target_h, double & angle);
  /**
   * @brief Get error message for the tool
   *
   * @return std::string
   */
  std::string error_message() {return error_message_;}

private:
  int max_iter_{20};
  ForwardMotionFunc forward_motion_func_;
  std::string error_message_;
};


}  // namespace rmoss_projectile_motion

#endif  // RMOSS_PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_
