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

#include "rmoss_util/task_manager.hpp"

#include <string>

namespace rmoss_util
{

TaskManager::TaskManager(
  rclcpp::Node::SharedPtr node,
  GetTaskStatusCallback get_task_status_callback,
  ControlTaskCallback control_task_callback)
{
  node_ = node;
  get_task_status_cb_ = get_task_status_callback;
  control_task_cb_ = control_task_callback;
  using namespace std::placeholders;
  get_task_status_srv_ = node_->create_service<rmoss_interfaces::srv::GetTaskStatus>(
    std::string(node_->get_name()) + "/get_task_status",
    std::bind(&TaskManager::get_task_status_cb, this, _1, _2));
  control_task_srv_ = node_->create_service<rmoss_interfaces::srv::ControlTask>(
    std::string(node_->get_name()) + "/control_task",
    std::bind(&TaskManager::control_task_cb, this, _1, _2));
}
void TaskManager::get_task_status_cb(
  const rmoss_interfaces::srv::GetTaskStatus::Request::SharedPtr request,
  rmoss_interfaces::srv::GetTaskStatus::Response::SharedPtr response)
{
  (void)request;
  auto status = get_task_status_cb_();
  response->status = static_cast<uint8_t>(status);
}
void TaskManager::control_task_cb(
  const rmoss_interfaces::srv::ControlTask::Request::SharedPtr request,
  rmoss_interfaces::srv::ControlTask::Response::SharedPtr response)
{
  (void)request;
  TaskCmd cmd = TaskCmd(request->cmd);
  response->success = control_task_cb_(cmd);
}

}  // namespace rmoss_util
