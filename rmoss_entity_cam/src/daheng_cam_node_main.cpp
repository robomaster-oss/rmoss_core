/*
 * @Author: holakk
 * @Date: 2021-11-05 22:51:41
 * @LastEditors: holakk
 * @LastEditTime: 2021-11-06 19:46:34
 * @Description: file content
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_entity_cam/daheng_cam_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rmoss_entity_cam::DaHengCamNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}