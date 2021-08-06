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

#ifndef RM_UTIL__MONO_MEASURE_TOOL_HPP_
#define RM_UTIL__MONO_MEASURE_TOOL_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_util
{
// 基于单目视觉位置测量。
class MonoMeasureTool
{
public:
  bool set_camera_info(std::vector<double> camera_intrinsic, std::vector<double> camera_distortion);
  ////////// 3d点坐标求解（use solve pnp）
  // points2d: input,一组图像上的2d点（4个点）
  // points3d: input,一组3d点（世界坐标系），对应图像上的点（4个点）
  // position: output,世界坐标系原点在相机坐标系下的位置。
  // return :state
  bool solve_pnp(
    std::vector<cv::Point2f> & points2d, std::vector<cv::Point3f> & points3d,
    cv::Point3f & position);
  ////// 逆投影，已知深度，2d->3d点求解
  // p: intput,图像上点坐标
  // distance: input,已知的真实距离
  // return :对应的真实3d点坐标
  cv::Point3f unproject(cv::Point2f p, double distance);
  ////// 视角求解
  // p: intput,图像上点坐标
  // pitch: output,视角pitch
  // yaw: output,视角yaw
  // return :state
  void calc_view_angle(cv::Point2f p, float & pitch, float & yaw);

private:
  // 相机参数
  cv::Mat camera_intrinsic_;   // 相机内参3*3
  cv::Mat camera_distortion_;  // 相机畸变参数1*5
};

}  // namespace rm_util

#endif  // RM_UTIL__MONO_MEASURE_TOOL_HPP_
