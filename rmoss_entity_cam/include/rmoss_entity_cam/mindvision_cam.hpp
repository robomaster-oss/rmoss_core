#ifndef RM_CAM__MINDVISION_CAM_HPP
#define RM_CAM__MINDVISION_CAM_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "CameraApi.h"
#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

namespace rmoss_entity_cam
{
    // MindVision相机类
    class MindVisionCam: public rmoss_cam::CamInterface
    {
    public:
        /** Construct
         * @param camera_sn 相机SN号
         * @param node ROS节点指针
         * @param config_path 相机配置文件路径
         **/
        explicit MindVisionCam(
            const std::string &camera_sn,
            rclcpp::Node::SharedPtr node,
            const std::string &config_path = ""
        );


        bool open() override;
        bool close() override;
        bool is_open() override;
        bool grab_image(cv::Mat &image) override;
        bool grab_image(cv::Mat &image, double &timestamp_ms) override;
        bool set_parameter(rmoss_cam::CamParamType type, int value) override;
        bool get_parameter(rmoss_cam::CamParamType type, int & value) override;
        std::string error_message() override;

        bool save_config(const std::string &path);

    private:
        bool read_camera_info();
        bool config_camera();

    private:
        std::string         camera_sn_;                 // 相机sn号
        
        rclcpp::Node::SharedPtr node_;
        std::string         config_path_;               // 相机配置文件路径

        int                 hCamera_;                   // 相机描述子
        tSdkCameraDevInfo   cameraList_[16];            // 相机列表(仅获取16个相机)
        tSdkCameraCapbility cameraInfo_;                // 相机特征信息

        BYTE*               pFrameBuffer_;              // RAW图像缓存
        bool                is_soft_trigger_{false};    // 是否软触发
        bool                is_open_;                   // 相机状态

        std::unordered_map<rmoss_cam::CamParamType, int> _param; // 摄像头参数
    };
} // namespace rm_cam


#endif // RM_CAM__MINDVISION_CAM_HPP