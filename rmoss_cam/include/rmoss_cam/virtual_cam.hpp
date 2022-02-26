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

#ifndef RMOSS_CAM__VIRTUAL_CAM_HPP_
#define RMOSS_CAM__VIRTUAL_CAM_HPP_

#include <string>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

namespace rmoss_cam
{
// the virtual camera device by using image or video, based on opencv
class VirtualCam : public CamInterface
{
public:
  enum { IMAGE_MODE, VIDEO_MODE};
  explicit VirtualCam(int mode, const std::string & path);

  bool open() override;
  bool close() override;
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(CamParamType type, int value) override;
  bool get_parameter(CamParamType type, int & value) override;
  std::string error_message() override {return error_message_;}

private:
  // for image
  cv::Mat img_;
  // for video
  cv::VideoCapture cap_;
  int total_frames_;
  int current_frame;
  // camera parameters
  std::unordered_map<CamParamType, int> params_;
  std::string init_error_message_;
  std::string error_message_;
  // flag
  bool init_ok_{false};
  bool is_open_{false};
  int current_mode_;
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__VIRTUAL_CAM_HPP_
