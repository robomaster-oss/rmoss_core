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

#include "rmoss_util/debug.hpp"

#include <vector>

namespace rmoss_util
{
bool g_is_debug = false;

bool get_debug() {return g_is_debug;}
void set_debug(bool get_debug) {g_is_debug = get_debug;}

// 绘制旋转矩形
void draw_rotated_rect(cv::Mat & img, cv::RotatedRect r, cv::Scalar color)
{
  cv::Point2f rect_points[4];
  r.points(rect_points);
  for (int j = 0; j < 4; j++) {
    cv::line(img, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8);
  }
}
// 绘制四边形
void draw_4points(cv::Mat & img, cv::Point2f * point2fs, cv::Scalar color)
{
  for (int j = 0; j < 4; j++) {
    cv::line(img, point2fs[j], point2fs[(j + 1) % 4], color, 1, 8);
  }
}
// 绘制多边形
void draw_convex_hull(cv::Mat & img, std::vector<cv::Point2f> & points, cv::Scalar color)
{
  for (size_t j = 0; j < points.size() - 1; j++) {
    cv::line(img, points[j], points[j + 1], color, 1, 8);
  }
  cv::line(img, points[0], points[points.size() - 1], color, 1, 8);
}

}  // namespace rmoss_util
