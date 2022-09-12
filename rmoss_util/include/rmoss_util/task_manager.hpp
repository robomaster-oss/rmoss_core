// Copyright 2021 robomaster-oss.
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

#ifndef RMOSS_UTIL__TASK_MANAGER_HPP_
#define RMOSS_UTIL__TASK_MANAGER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/srv/get_task_status.hpp"
#include "rmoss_interfaces/srv/control_task.hpp"

namespace rmoss_util
{

enum class TaskStatus : uint8_t {Running = 1, Idle, Error};
enum class TaskCmd : uint8_t {Start = 1, Stop};

class TaskManager
{
  typedef std::function<enum TaskStatus ()> GetTaskStatusCallback;
  typedef std::function<bool (TaskCmd)> ControlTaskCallback;

public:
  using SharedPtr = std::shared_ptr<TaskManager>;
  TaskManager(
    rclcpp::Node::SharedPtr node,
    GetTaskStatusCallback get_task_status_callback,
    ControlTaskCallback control_task_callback);
  void get_task_status_cb(
    const rmoss_interfaces::srv::GetTaskStatus::Request::SharedPtr request,
    rmoss_interfaces::srv::GetTaskStatus::Response::SharedPtr response);
  void control_task_cb(
    const rmoss_interfaces::srv::ControlTask::Request::SharedPtr request,
    rmoss_interfaces::srv::ControlTask::Response::SharedPtr response);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<rmoss_interfaces::srv::GetTaskStatus>::SharedPtr get_task_status_srv_;
  rclcpp::Service<rmoss_interfaces::srv::ControlTask>::SharedPtr control_task_srv_;
  // callback
  GetTaskStatusCallback get_task_status_cb_;
  ControlTaskCallback control_task_cb_;
};

}  // namespace rmoss_util

#endif  // RMOSS_UTIL__TASK_MANAGER_HPP_
