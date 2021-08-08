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

#ifndef RM_CAM__VIRTUAL_CAM_HPP_
#define RM_CAM__VIRTUAL_CAM_HPP_

#include <string>

#include "opencv2/opencv.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{
// the virtual camera device by using image or video, based on opencv
class VirtualCam : public CamInterface
{
public:
  enum { IMAGE_MODE, VIDEO_MODE};
  explicit VirtualCam(int mode, const std::string & path);

  bool open() override;
  void close() override {}
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(CamParamType type, int value) override;
  bool get_parameter(CamParamType type, int & value) override;

private:
  // for image
  std::string image_path_;
  cv::Mat img_;
  // for video
  std::string video_path_;
  cv::VideoCapture cap_;
  int total_frames_;
  int current_frame;
  // camera para
  int cam_width_;
  int cam_height_;
  int cam_fps_;
  // flag
  bool is_open_ = false;
  int current_mode_;
};

}  // namespace rm_cam

#endif  // RM_CAM__VIRTUAL_CAM_HPP_
