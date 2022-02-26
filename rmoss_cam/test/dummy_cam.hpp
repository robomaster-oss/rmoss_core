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
#include <unordered_map>
#include <thread>

#include "rmoss_cam/cam_interface.hpp"

// 用于测试。
class DummyCam : public rmoss_cam::CamInterface
{
public:
  explicit DummyCam(int grap_time_ms = 10)
  : grap_time_ms_(grap_time_ms)
  {
    params_[rmoss_cam::CamParamType::Width] = 640;
    params_[rmoss_cam::CamParamType::Height] = 480;
  }
  bool open() override
  {
    int h = params_[rmoss_cam::CamParamType::Height];
    int w = params_[rmoss_cam::CamParamType::Width];
    img_ = cv::Mat(h, w, CV_8UC3, cv::Scalar(0));
    is_falut_ = false;
    is_open_ = true;
    return true;
  }
  bool close() override
  {
    img_.release();
    is_open_ = false;
    return true;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(grap_time_ms_));
      image = img_.clone();
      return true;
    }
    return false;
  }
  bool set_parameter(rmoss_cam::CamParamType type, int value) override
  {
    if (params_.find(type) != params_.end()) {
      params_[type] = value;
      return true;
    } else {
      return false;
    }
  }
  bool get_parameter(rmoss_cam::CamParamType type, int & value) override
  {
    if (params_.find(type) != params_.end()) {
      value = params_[type];
      return true;
    } else {
      return false;
    }
  }
  std::string error_message() override
  {
    return error_message_;
  }
  void set_falut()
  {
    is_falut_ = true;
  }

private:
  // for image
  cv::Mat img_;
  int grap_time_ms_{10};
  // camera parameters
  std::unordered_map<rmoss_cam::CamParamType, int> params_;
  std::string error_message_;
  // flag
  bool is_open_{false};
  bool is_falut_{false};
};

#endif  // DUMMY_CAM_HPP_
