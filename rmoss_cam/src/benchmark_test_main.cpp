// Copyright 2022 RoboMaster-OSS
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

#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_cam/virtual_cam_node.hpp"
#include "rmoss_cam/cam_client.hpp"

using namespace std::chrono_literals;

template<typename NodeT>
std::shared_ptr<std::thread> create_spin_thread(NodeT & node)
{
  return std::make_shared<std::thread>(
    [node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
      executor.remove_node(node->get_node_base_interface());
    });
}

std::shared_ptr<rmoss_cam::VirtualCamNode> create_cam_node(
  const std::string & node_name,
  const std::string & camera_name,
  bool use_intra_process_comms = false)
{
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms);
  node_options.arguments({"--ros-args", "-r", std::string("__node:=") + node_name, "--"});
  node_options.append_parameter_override("camera_name", camera_name);
  return std::make_shared<rmoss_cam::VirtualCamNode>(node_options);
}

void benchmark_test(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rmoss_cam::CamClient> cam_client,
  std::string camera_name)
{
  int num = 0;
  double first_time;
  double last_time;
  double delay_sum = 0;
  bool ok{false};
  cam_client->set_camera_name(camera_name);
  cam_client->set_camera_callback(
    [&](const cv::Mat & /*img*/, const rclcpp::Time & stamp) {
      if (num >= 500) {
        return;
      }
      num++;
      if (num == 1) {
        first_time = stamp.seconds();
      } else if (num == 500) {
        last_time = stamp.seconds();
        ok = true;
      }
      auto delay = (node->get_clock()->now().seconds() - stamp.seconds());
      delay_sum = delay_sum + delay;
    });
  cam_client->connect();
  // wait and disconnect
  while (!ok) {
    std::this_thread::sleep_for(100ms);
  }
  cam_client->disconnect();
  // print result
  std::cout << "result for <" + camera_name << ">:" << std::endl;
  std::cout << "fps:" << 1.0 * num / (last_time - first_time) << std::endl;
  std::cout << "delay:" << delay_sum / num << std::endl;
}

int main(int argc, char * argv[])
{
  // create ros2 node
  rclcpp::init(argc, argv);
  // spin threads
  std::vector<std::shared_ptr<std::thread>> threads;
  // benchmark node
  auto node = std::make_shared<rclcpp::Node>("benchmark");
  // cam1: normal node (run by launch file)
  // cam2: composed node
  auto cam2_node = create_cam_node("virtual_cam2", "benchmark_cam2");
  threads.push_back(create_spin_thread(cam2_node));
  // cam3: composed node with rclcpp intra-comms
  auto cam3_node = create_cam_node("virtual_cam3", "benchmark_cam3", true);
  threads.push_back(create_spin_thread(cam3_node));
  // create camera clients
  auto client_node1 = std::make_shared<rclcpp::Node>("client_node1");
  auto cam_client1 = std::make_shared<rmoss_cam::CamClient>(client_node1);
  auto client_node2 = std::make_shared<rclcpp::Node>("client_node2");
  auto cam_client2 = std::make_shared<rmoss_cam::CamClient>(client_node2);
  auto client_node3 = std::make_shared<rclcpp::Node>(
    "client_node3",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto cam_client3 = std::make_shared<rmoss_cam::CamClient>(client_node3);
  // benchmark test
  std::this_thread::sleep_for(1s);
  std::cout << "start test for normal multi-process" << std::endl;
  benchmark_test(client_node1, cam_client1, "benchmark_cam1");
  std::this_thread::sleep_for(1s);
  std::cout << "start test for normal composition" << std::endl;
  benchmark_test(client_node2, cam_client2, "benchmark_cam2");
  std::this_thread::sleep_for(1s);
  std::cout << "start test for composition with rclcpp intra-comms" << std::endl;
  benchmark_test(client_node3, cam_client3, "benchmark_cam3");
  // wait end
  std::this_thread::sleep_for(1s);
  rclcpp::shutdown();
  for (auto t : threads) {
    t->join();
  }
  return 0;
}
