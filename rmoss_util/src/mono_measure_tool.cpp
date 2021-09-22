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

#include "rmoss_util/mono_measure_tool.hpp"

#include <vector>

namespace rmoss_util
{
bool MonoMeasureTool::set_camera_info(
  std::vector<double> camera_intrinsic, std::vector<double> camera_distortion)
{
  if (camera_intrinsic.size() != 9) {
    // the size of camera intrinsic must be 9 (equal 3*3)
    return false;
  }
  // init camera_intrinsic and camera_distortion
  cv::Mat camera_intrinsic_mat(camera_intrinsic, true);
  camera_intrinsic_mat = camera_intrinsic_mat.reshape(0, 3);
  camera_intrinsic_ = camera_intrinsic_mat.clone();
  //
  cv::Mat camera_distortion_mat(camera_distortion, true);
  camera_distortion_mat = camera_distortion_mat.reshape(0, 1);
  camera_distortion_ = camera_distortion_mat.clone();
  return true;
}

bool MonoMeasureTool::solve_pnp(
  std::vector<cv::Point2f> & points2d, std::vector<cv::Point3f> & points3d, cv::Point3f & position)
{
  if (points2d.size() != points3d.size()) {
    return false;  // 投影点数量不匹配
  }
  // cv::Mat rot = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat trans = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat r;  // 旋转向量
  solvePnP(points3d, points2d, camera_intrinsic_, camera_distortion_, r, trans);
  position = cv::Point3f(trans);
  return true;
}

// refer to :http://www.cnblogs.com/singlex/p/pose_estimation_1_1.html
// 根据输入的参数将图像坐标转换到相机坐标中
// 输入为图像上的点坐标
// double distance 物距
// 输出3d点坐标的单位与distance（物距）的单位保持一致
cv::Point3f MonoMeasureTool::unproject(cv::Point2f p, double distance)
{
  auto fx = camera_intrinsic_.ptr<double>(0)[0];
  auto u0 = camera_intrinsic_.ptr<double>(0)[2];
  auto fy = camera_intrinsic_.ptr<double>(1)[1];
  auto v0 = camera_intrinsic_.ptr<double>(1)[2];

  double zc = distance;
  double xc = (p.x - u0) * distance / fx;
  double yc = (p.y - v0) * distance / fy;
  return cv::Point3f(xc, yc, zc);
}

// 获取image任意点的视角，pitch，yaw（相对相机坐标系）。
// 与相机坐标系保持一致。
void MonoMeasureTool::calc_view_angle(cv::Point2f p, float & pitch, float & yaw)
{
  auto fx = camera_intrinsic_.ptr<double>(0)[0];
  auto u0 = camera_intrinsic_.ptr<double>(0)[2];
  auto fy = camera_intrinsic_.ptr<double>(1)[1];
  auto v0 = camera_intrinsic_.ptr<double>(1)[2];

  pitch = atan2((p.y - v0), fy);
  yaw = atan2((p.x - u0), fx);
}

}  // namespace rmoss_util
