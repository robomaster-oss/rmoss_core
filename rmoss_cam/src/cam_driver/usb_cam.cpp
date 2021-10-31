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

#include "rmoss_cam/usb_cam.hpp"

#include <string>

namespace rmoss_cam
{

UsbCam::UsbCam(const std::string & dev_path)
: dev_path_(dev_path)
{
  params_[CamParamType::Fps] = 30;
  params_[CamParamType::Width] = 640;
  params_[CamParamType::Height] = 480;
}

UsbCam::~UsbCam()
{
  if (is_open_) {
    cap_.release();
  }
}

bool UsbCam::open()
{
  if (is_open_) {
    return true;
  }
  // open device
  if (cap_.open(dev_path_)) {
    // success to open
    cap_.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, params_[CamParamType::Width]);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, params_[CamParamType::Height]);
    is_open_ = true;
    return true;
  } else {
    // fail to open
    return false;
  }
}

void UsbCam::close()
{
  if (is_open_) {
    cap_.release();
    is_open_ = false;
  }
}

bool UsbCam::is_open()
{
  return is_open_;
}

bool UsbCam::grab_image(cv::Mat & image)
{
  if (cap_.isOpened()) {
    if (cap_.read(image)) {
      return true;
    }
  }
  return false;
}

// set and get parameter
bool UsbCam::set_parameter(CamParamType type, int value)
{
  if (params_.find(type) != params_.end()) {
    params_[type] = value;
    return true;
  } else {
    return false;
  }
}
bool UsbCam::get_parameter(CamParamType type, int & value)
{
  if (params_.find(type) != params_.end()) {
    value = params_[type];
    return true;
  } else {
    return false;
  }
}

}  // namespace rmoss_cam
