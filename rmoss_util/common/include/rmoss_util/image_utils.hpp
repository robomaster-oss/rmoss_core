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

#ifndef RMOSS_UTIL__IMAGE_UTILS_HPP_
#define RMOSS_UTIL__IMAGE_UTILS_HPP_

#include <vector>

#include "opencv2/opencv.hpp"

namespace rmoss_util
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

// const definition of color
const auto blue = cv::Scalar(255, 0, 0);
const auto green = cv::Scalar(0, 255, 0);
const auto red = cv::Scalar(0, 0, 255);

// math function
float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
bool calc_circle_from_3points(
  cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f & point, float & r);

// draw function
void draw_rotated_rect(cv::Mat & img, cv::RotatedRect r, cv::Scalar color = green);
void draw_4points(cv::Mat & img, cv::Point2f * point2fs, cv::Scalar color = green);
void draw_convex_hull(cv::Mat & img, std::vector<cv::Point2f> & points, cv::Scalar color = green);
}  // namespace rmoss_util

#endif  // RMOSS_UTIL__IMAGE_UTILS_HPP_
