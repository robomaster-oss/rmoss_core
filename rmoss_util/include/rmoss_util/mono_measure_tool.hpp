// Copyright 2020 robomaster-oss.
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

#ifndef RMOSS_UTIL__MONO_MEASURE_TOOL_HPP_
#define RMOSS_UTIL__MONO_MEASURE_TOOL_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

namespace rmoss_util
{
// 基于单目视觉位置测量。
class MonoMeasureTool
{
public:
  /**
   * @brief Set the camera intrinsic parameter
   *
   * @param camera_intrinsic camera intrinsic in 3x3 matrix flat in line stretch
   * @param camera_distortion camera distortion parameter in plumb_bob distortion model
   * @return true
   * @return false
   */
  bool set_camera_info(std::vector<double> camera_intrinsic, std::vector<double> camera_distortion);
  /**
   * @brief Solve Perspective-n-Point problem in camera
   * 3d点坐标求解（use solve pnp）
   * @param points2d a list of points in image frame
   * @param points3d a list of points correspondend to points2d
   * @param position output position of the origin point of 3d coordinate system
   * @return true
   * @return false
   */
  bool solve_pnp(
    std::vector<cv::Point2f> & points2d, std::vector<cv::Point3f> & points3d,
    cv::Point3f & position);
  /**
   * @brief 逆投影，已知深度，2d->3d点求解
   *
   * @param p 图像上点坐标
   * @param distance 已知的真实距离
   * @return cv::Point3f 对应的真实3d点坐标
   */
  cv::Point3f unproject(cv::Point2f p, double distance);
  /**
   * @brief 视角求解
   *
   * @param p 图像上点坐标
   * @param pitch 视角pitch
   * @param yaw 视角yaw
   */
  void calc_view_angle(cv::Point2f p, float & pitch, float & yaw);

private:
  // 相机参数
  cv::Mat camera_intrinsic_;   // 相机内参3*3
  cv::Mat camera_distortion_;  // 相机畸变参数1*5
};

}  // namespace rmoss_util

#endif  // RMOSS_UTIL__MONO_MEASURE_TOOL_HPP_
