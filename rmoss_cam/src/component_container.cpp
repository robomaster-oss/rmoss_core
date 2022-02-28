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
#include <vector>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/split.hpp"
#include "rmoss_cam/cam_server_manager.hpp"
#include "rmoss_cam/cam_node_factory.hpp"

// TODO(gezp) :use component_manager_isolated in the next ROS LTS distro Humble.
class CamComponentManager : public rclcpp_components::ComponentManager
{
public:
  CamComponentManager(
    std::weak_ptr<rclcpp::Executor> executor,
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false))
  : rclcpp_components::ComponentManager::ComponentManager(executor, node_name, node_options)
  {
    cam_server_manager_ = std::make_shared<rmoss_cam::CamServerManager>(
      this->get_node_logging_interface());
  }

protected:
  std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource & resource) override
  {
    std::string library_path = resource.second;
    std::string class_name = resource.first;
    std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";
    std::string cam_class_name = "rmoss_cam::CamNodeFactoryTemplate<" + class_name + ">";
    class_loader::ClassLoader * loader;
    if (loaders_.find(library_path) == loaders_.end()) {
      RCLCPP_INFO(get_logger(), "Load Library: %s", library_path.c_str());
      try {
        loaders_[library_path] = std::make_unique<class_loader::ClassLoader>(library_path);
      } catch (const std::exception & ex) {
        throw rclcpp_components::ComponentManagerException(
                "Failed to load library: " + std::string(ex.what()));
      } catch (...) {
        throw rclcpp_components::ComponentManagerException("Failed to load library");
      }
    }
    loader = loaders_[library_path].get();
    // try to load Camera Node (CamServer, CamClient)
    auto classes = loader->getAvailableClasses<rmoss_cam::CamNodeFactory>();
    for (const auto & clazz : classes) {
      RCLCPP_INFO(get_logger(), "Found class related to Camera: %s", clazz.c_str());
      if (clazz == cam_class_name) {
        RCLCPP_INFO(get_logger(), "Instantiate class: %s", clazz.c_str());
        auto result = loader->createInstance<rmoss_cam::CamNodeFactory>(clazz);
        result->set_resource_manager(cam_server_manager_);
        return result;
      }
    }
    // try to load normal Node
    auto classes2 = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (const auto & clazz : classes2) {
      RCLCPP_INFO(get_logger(), "Found class: %s", clazz.c_str());
      if (clazz == class_name || clazz == fq_class_name) {
        RCLCPP_INFO(get_logger(), "Instantiate class: %s", clazz.c_str());
        return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      }
    }
    return {};
  }

private:
  std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders_;
  std::shared_ptr<rmoss_cam::CamServerManager> cam_server_manager_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<CamComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}
