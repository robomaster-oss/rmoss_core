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

#include "rm_cam/usb_cam.hpp"

#include <string>

namespace rm_cam
{


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
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
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

bool UsbCam::set_parameter(CamParamType type, int value)
{
  if (is_open_) {
    return false;
  }
  if (type == CamParamType::Width) {
    cam_width_ = value;
    return true;
  } else if (type == CamParamType::Height) {
    cam_width_ = value;
    return true;
  } else if (type == CamParamType::Fps) {
    cam_fps_ = value;
    return true;
  }
  return false;
}

bool UsbCam::get_parameter(CamParamType type, int & value)
{
  switch (type) {
    case CamParamType::Width:
      value = cam_width_;
      return true;
    case CamParamType::Height:
      value = cam_height_;
      return true;
    case CamParamType::Exposure:
      return false;
    case CamParamType::Fps:
      value = cam_fps_;
      return true;
    default:
      return false;
  }
}

bool UsbCam::set_exposure(int value)
{
  // TODO(gezp): setting of exposure isn't supported in OpenCV
  (void)(value);
  return false;
}

}  // namespace rm_cam
