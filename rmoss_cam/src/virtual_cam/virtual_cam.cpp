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

#include "rmoss_cam/virtual_cam.hpp"

#include <string>

namespace rmoss_cam
{

VirtualCam::VirtualCam(int mode, const std::string & path)
{
  current_mode_ = mode;
  if (mode == IMAGE_MODE) {
    img_ = cv::imread(path);
    if (img_.empty()) {
      init_error_message_ = path + "is invalid";
      return;
    }
    params_[CamParamType::Fps] = 30;
    params_[CamParamType::Width] = img_.cols;
    params_[CamParamType::Height] = img_.rows;
    init_ok_ = true;
  } else if (mode == VIDEO_MODE) {
    if (!cap_.open(path)) {
      init_error_message_ = path + "is invalid";
      return;
    }
    params_[CamParamType::Width] = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    params_[CamParamType::Height] = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    params_[CamParamType::Fps] = cap_.get(cv::CAP_PROP_FPS);
    total_frames_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    init_ok_ = true;
  }
  init_error_message_ = "unknow mode (" + std::to_string(mode) + ")";
}

bool VirtualCam::open()
{
  if (is_open_) {
    return true;
  }
  if (!init_ok_) {
    error_message_ = init_error_message_;
    return false;
  }
  is_open_ = true;
  return true;
}

bool VirtualCam::close()
{
  if (is_open_) {
    is_open_ = false;
  }
  return true;
}

bool VirtualCam::is_open() {return is_open_;}

bool VirtualCam::grab_image(cv::Mat & image)
{
  if (!is_open_) {
    error_message_ = "camera is not open";
    return false;
  }
  if (current_mode_ == IMAGE_MODE) {
    image = img_.clone();
    return true;
  } else if (current_mode_ == VIDEO_MODE) {
    if (!cap_.read(image)) {
      error_message_ = "cv::VideoCapture.read() error";
      return false;
    }
    current_frame++;
    if (current_frame > total_frames_ - 2) {
      current_frame = 0;
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
    return true;
  }
  error_message_ = "unknow mode";
  return false;
}

// set and get parameter
bool VirtualCam::set_parameter(CamParamType type, int value)
{
  if (params_.find(type) != params_.end()) {
    params_[type] = value;
    return true;
  } else {
    error_message_ = "";
    return false;
  }
}

bool VirtualCam::get_parameter(CamParamType type, int & value)
{
  if (params_.find(type) != params_.end()) {
    value = params_[type];
    return true;
  } else {
    error_message_ = "";
    return false;
  }
}

}  // namespace rmoss_cam
