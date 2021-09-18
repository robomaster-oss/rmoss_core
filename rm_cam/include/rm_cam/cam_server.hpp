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

#ifndef RM_CAM__CAM_SERVER_HPP_
#define RM_CAM__CAM_SERVER_HPP_

#include <thread>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "rmoss_interfaces/srv/get_camera_info.hpp"
#include "rm_cam/cam_interface.hpp"

namespace rm_cam
{
// camera task wrapper to publish image
class CamServer
{
public:
  CamServer(
    rclcpp::Node * node,
    std::shared_ptr<CamInterface> cam_intercace);

  // bool set_camera_

private:
  void timer_callback();
  void camera_info_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const rmoss_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
    rmoss_interfaces::srv::GetCameraInfo::Response::SharedPtr response);

private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  image_transport::Publisher img_pub_;
  rclcpp::Service<rmoss_interfaces::srv::GetCameraInfo>::SharedPtr camera_info_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  // camera_device interface
  std::shared_ptr<CamInterface> cam_intercace_;
  // data
  cv::Mat img_;
  std::vector<double> camera_k_;  // 3*3=9
  std::vector<double> camera_p_;  // 3*4=12
  std::vector<double> camera_d_;
  bool has_camera_info_{false};
  int fps_{30};
  int reopen_cnt{0};
};

}  // namespace rm_cam

#endif  // RM_CAM__CAM_SERVER_HPP_
