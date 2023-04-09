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

#ifndef RMOSS_CAM__CAM_SERVER_HPP_
#define RMOSS_CAM__CAM_SERVER_HPP_

#include <thread>
#include <string>
#include <memory>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rmoss_interfaces/srv/get_camera_info.hpp"
#include "rmoss_cam/cam_interface.hpp"
#include "rmoss_util/task_manager.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"

namespace rmoss_cam
{
// camera server wrapper to publish image
class CamServer
{
public:
  typedef std::function<void (const cv::Mat &, const rclcpp::Time &)> Callback;
  /**
   * @brief Construct a new Cam Server object
   *
   * @param node node interface for rclcpp
   * @param cam_intercace camera interface
   */
  CamServer(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<CamInterface> cam_intercace);

  /**
   * @brief Get the camera name
   *
   * @return std::string
   */
  std::string get_camera_name() {return camera_name_;}

private:
  void init_timer();
  void init_task_manager();
  void get_camera_info_cb(
    const rmoss_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
    rmoss_interfaces::srv::GetCameraInfo::Response::SharedPtr response);

private:
  rclcpp::Node::SharedPtr node_;

  // default image transport
  std::shared_ptr<image_transport::Publisher> img_pub_;
  // image_transporter for camera publisher
  std::shared_ptr<image_transport::CameraPublisher> cam_pub_;

  rclcpp::Service<rmoss_interfaces::srv::GetCameraInfo>::SharedPtr get_camera_info_srv_;
  rmoss_util::TaskManager::SharedPtr task_manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  // camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  // camera_device interface
  std::shared_ptr<CamInterface> cam_intercace_;
  // params
  std::string camera_name_{"camera"};
  std::string camera_frame_id_{""};
  bool use_qos_profile_sensor_data_{false};
  bool use_image_transport_camera_publisher_{false};
  bool run_flag_{false};
  bool cam_status_ok_{false};
  // data
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
  cv::Mat img_;
  int fps_{30};
  int reopen_cnt{0};
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__CAM_SERVER_HPP_
