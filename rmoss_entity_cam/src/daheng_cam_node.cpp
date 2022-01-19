/*
 * @Author: holakk
 * @Date: 2021-11-05 22:51:26
 * @LastEditors: holakk
 * @LastEditTime: 2021-11-10 17:02:00
 * @Description: file content
 */

#include "rmoss_entity_cam/daheng_cam_node.hpp"

namespace rmoss_entity_cam
{
    DaHengCamNode::DaHengCamNode(const rclcpp::NodeOptions &options)
    {
        this->node_ = std::make_shared<rclcpp::Node>("daheng_cam", options);

        this->node_->declare_parameter("sn", "");
        this->node_->declare_parameter("config_path", "");
        this->node_->declare_parameter("lut_config_path", "");
        this->node_->declare_parameter("lut_detail", std::vector<double>({0., 2, 0.}));

        std::string cam_sn = this->node_->get_parameter("sn").as_string();
        std::string config_path = this->node_->get_parameter("config_path").as_string();
        std::string lut_config_path = this->node_->get_parameter("lut_config_path").as_string();
        std::vector<double> lut_detail = this->node_->get_parameter("lut_detail").as_double_array();

        cam_dev_ = std::make_shared<DaHengCam>(cam_sn, this->node_, config_path, lut_config_path, lut_detail);
        if (!this->cam_dev_->open())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Try to open camera or load config failed!");
            return;
        }
        if (!this->cam_dev_->close())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Try to close camera failed!");
            return;
        }
        cam_server_ = std::make_shared<rmoss_cam::CamServer>(this->node_, this->cam_dev_);
    }
} // namespace rm_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_entity_cam::DaHengCamNode)
