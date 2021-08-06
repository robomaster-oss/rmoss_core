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

#ifndef RM_UTIL__MATH_HPP_
#define RM_UTIL__MATH_HPP_

#include "opencv2/opencv.hpp"

namespace rm_util
{
template<typename T>
inline T rad_to_deg(T radian)
{
  return radian * 180 / CV_PI;
}
template<typename T>
inline T deg_to_rad(T degree)
{
  return degree / 180 * CV_PI;
}
float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
bool calc_circle_from_3points(
  cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f & point, float & r);
}  // namespace rm_util

#endif  // RM_UTIL__MATH_HPP_
