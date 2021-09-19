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

#ifndef DUMMY_CAM_HPP_
#define DUMMY_CAM_HPP_

#include <memory>
#include <string>

#include "rm_cam/cam_interface.hpp"

// 用于测试。
class DummyCam : public rm_cam::CamInterface
{
public:
  DummyCam()
  {
    params_[rm_cam::CamParamType::Width] = 640;
    params_[rm_cam::CamParamType::Height] = 480;
  }
  bool open() override
  {
    int h = params_[rm_cam::CamParamType::Height];
    int w = params_[rm_cam::CamParamType::Width];
    img_ = cv::Mat(h, w, CV_8UC3, cv::Scalar(0));
    is_falut_ = false;
    is_open_ = true;
    return true;
  }
  void close() override
  {
    img_.release();
    is_open_ = false;
  }
  bool is_open() override
  {
    return is_open_;
  }
  bool grab_image(cv::Mat & image) override
  {
    if (is_open_) {
      if (is_falut_) {
        return false;
      }
      image = img_.clone();
      return true;
    }
    return false;
  }

  void set_falut()
  {
    is_falut_ = true;
  }

private:
  // for image
  cv::Mat img_;
  // flag
  bool is_open_{false};
  bool is_falut_{false};
};

#endif  // DUMMY_CAM_HPP_
