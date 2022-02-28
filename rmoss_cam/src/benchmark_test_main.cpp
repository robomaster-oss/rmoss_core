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

#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_cam/virtual_cam_node.hpp"
#include "rmoss_cam/intra_cam_client.hpp"

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
  cam_client->connect(
    camera_name,
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
  auto cam_server_manager = std::make_shared<rmoss_cam::CamServerManager>(
    node->get_node_logging_interface());
  // cam1: normal node (run by launch file)
  // cam2: composed node
  auto node_options2 = rclcpp::NodeOptions();
  node_options2.arguments({"--ros-args", "-r", std::string("__node:=") + "virtual_cam2", "--"});
  node_options2.append_parameter_override("camera_name", "benchmark_cam2");
  auto cam2_node = std::make_shared<rmoss_cam::VirtualCamNode>(node_options2);
  threads.push_back(create_spin_thread(cam2_node));
  // cam3: composed node (intra-comunication)
  auto node_options3 = rclcpp::NodeOptions();
  node_options3.arguments({"--ros-args", "-r", std::string("__node:=") + "virtual_cam3", "--"});
  node_options3.append_parameter_override("camera_name", "benchmark_cam3");
  auto cam3_node = std::make_shared<rmoss_cam::VirtualCamNode>(node_options3);
  cam3_node->set_resource_manager(cam_server_manager);
  threads.push_back(create_spin_thread(cam3_node));
  // create camera clients
  auto client_node1 = std::make_shared<rclcpp::Node>("client_node1");
  auto cam_client1 = std::make_shared<rmoss_cam::CamClient>(client_node1);
  auto client_node2 = std::make_shared<rclcpp::Node>("client_node2");
  auto cam_client2 = std::make_shared<rmoss_cam::CamClient>(client_node2);
  auto client_node3 = std::make_shared<rclcpp::Node>("client_node3");
  auto cam_client3 = std::make_shared<rmoss_cam::IntraCamClient>(client_node3, cam_server_manager);
  // benchmark test
  std::this_thread::sleep_for(2s);
  std::cout << "start test for normal process" << std::endl;
  benchmark_test(client_node1, cam_client1, "benchmark_cam1");
  std::this_thread::sleep_for(2s);
  std::cout << "start test for normal component" << std::endl;
  benchmark_test(client_node2, cam_client2, "benchmark_cam2");
  std::this_thread::sleep_for(2s);
  std::cout << "start test for intra-component" << std::endl;
  benchmark_test(client_node3, cam_client3, "benchmark_cam3");
  // wait end
  for (auto t : threads) {
    t->join();
  }
  rclcpp::shutdown();
  return 0;
}
