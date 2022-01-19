#pragma once
#ifndef RM_CAM__DAHENG_CAM_NODE_HPP
#define RM_CAM__DAHENG_CAM_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/cam_server.hpp"
#include "rmoss_entity_cam/daheng_cam.hpp"

namespace rmoss_entity_cam
{
    // Node warpper for DaHengCamera
    class DaHengCamNode
    {
    public:
        explicit DaHengCamNode(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<DaHengCam> cam_dev_;
        std::shared_ptr<rmoss_cam::CamServer> cam_server_;
    };
}

#endif // RM_CAM__DAHENG_CAM_NODE_HPP