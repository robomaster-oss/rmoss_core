/*
 * @Author: holakk
 * @Date: 2021-11-10 17:59:31
 * @LastEditors: holakk
 * @LastEditTime: 2021-11-10 18:00:42
 * @Description: file content
 */
#include "rmoss_entity_cam/mindvision_cam_node.hpp"

#include <memory>
#include <string>

namespace rmoss_entity_cam
{
    MindVisionCamNode::MindVisionCamNode(
        const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("mv_cam", options);

        node_->declare_parameter("sn", "");
        node_->declare_parameter("config_path", "");

        std::string cam_sn = node_->get_parameter("sn").as_string();
        std::string config_path = node_->get_parameter("config_path").as_string();

        RCLCPP_INFO(
            node_->get_logger(),
            "Try to open camera at sn:%s", cam_sn.c_str());
        RCLCPP_INFO(
            node_->get_logger(),
            "Camera config file path:%s", config_path.c_str());

        cam_dev_ = std::make_shared<MindVisionCam>(cam_sn, node_, config_path);

        if (config_path == "")
        {
            if (!cam_dev_->open())
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Try to open camera failed!");
                return;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Writing camera config to file.");
            if (!cam_dev_->save_config(cam_sn + ".config"))
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Write failed!");
                return;
            }
            cam_dev_->close();
        }

        // create server
        cam_server_ = std::make_shared<rmoss_cam::CamServer>(this->node_, this->cam_dev_);
    }
} // namespace rm_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_entity_cam::MindVisionCamNode)
