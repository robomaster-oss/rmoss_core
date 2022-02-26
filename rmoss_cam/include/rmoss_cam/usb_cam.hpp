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
#ifndef RMOSS_CAM__USB_CAM_HPP_
#define RMOSS_CAM__USB_CAM_HPP_

#include <string>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

namespace rmoss_cam
{
// the usb camera (UVC) device, based on opencv
class UsbCam : public CamInterface
{
public:
  explicit UsbCam(const std::string & dev_path);
  ~UsbCam();

public:
  bool open() override;
  bool close() override;
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(CamParamType type, int value) override;
  bool get_parameter(CamParamType type, int & value) override;
  std::string error_message() override {return error_message_;}

private:
  std::string dev_path_;
  std::string error_message_;
  cv::VideoCapture cap_;
  // camera parameters
  std::unordered_map<CamParamType, int> params_;
  // flag
  bool is_open_{false};
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__USB_CAM_HPP_
