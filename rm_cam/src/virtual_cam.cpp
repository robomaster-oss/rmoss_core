// Copyright 2020 RoboMaster-OSS
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

#include "rm_cam/virtual_cam.hpp"

#include <string>

namespace rm_cam
{

VirtualCam::VirtualCam(int mode, const std::string & path)
{
  if (mode == IMAGE_MODE) {
    image_path_ = path;
    current_mode_ = IMAGE_MODE;
  } else {
    video_path_ = path;
    current_mode_ = VIDEO_MODE;
  }
}

bool VirtualCam::open()
{
  if (current_mode_ == IMAGE_MODE) {
    img_ = cv::imread(image_path_);
    if (!img_.empty()) {
      cam_width_ = img_.cols;
      cam_height_ = img_.rows;
      is_open_ = true;
      return true;
    }
  } else if (current_mode_ == VIDEO_MODE) {
    if (cap_.open(video_path_)) {
      cam_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
      cam_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
      total_frames_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
      cam_fps_ = cap_.get(cv::CAP_PROP_FPS);
      is_open_ = true;
      return true;
    }
  }
  return false;
}

bool VirtualCam::is_open() {return is_open_;}

bool VirtualCam::grab_image(cv::Mat & image)
{
  if (is_open_) {
    if (current_mode_ == IMAGE_MODE) {
      image = img_.clone();
      return true;
    } else if (current_mode_ == VIDEO_MODE) {
      if (cap_.read(image)) {
        current_frame++;
        if (current_frame > total_frames_ - 2) {
          current_frame = 0;
          cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        return true;
      }
    }
  }
  return false;
}

bool VirtualCam::set_parameter(CamParamType type, int value)
{
  if (is_open_) {
    return false;
  }
  if (type == CamParamType::Width) {
    return cam_width_ == value;
  } else if (type == CamParamType::Height) {
    return cam_width_ == value;
  } else if (type == CamParamType::Fps) {
    cam_fps_ = value;
    return true;
  }
  return false;
}

bool VirtualCam::get_parameter(CamParamType type, int & value)
{
  switch (type) {
    case CamParamType::Width:
      value = cam_width_;
      return true;
    case CamParamType::Height:
      value = cam_height_;
      return true;
    case CamParamType::Fps:
      value = cam_fps_;
      return true;
    default:
      return false;
  }
}


}  // namespace rm_cam
