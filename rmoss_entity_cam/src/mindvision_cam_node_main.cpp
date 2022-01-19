#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_entity_cam/mindvision_cam_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rmoss_entity_cam::MindVisionCamNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}