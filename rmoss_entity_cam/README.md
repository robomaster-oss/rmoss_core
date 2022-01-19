# rmoss_entity_cam模块

## 简介

`rmoss_entity_cam`提供了大恒和MindVision USB3.0工业相机ROS节点相关功能。

文件说明：

* `daheng_cam.hpp.hpp/cpp` : 大恒相机设备实现。
* `mindvision_cam.hpp.hpp/cpp`：MindVision相机设备实现。
* `mindvision_cam_node.hpp/cpp` ，`daheng_cam_node.hpp/cpp` :  ROS顶层模块（基于`DaHengCam`,`MindVisionCam`和`CamServer`），实现大恒相机节点和MindVision相机节点。

## 使用方式

### 大恒相机

> 参照`rmoss_cam`工具包中定义

launch方式运行：

```bash
ros2 launch rmoss_entity_cam daheng_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)

### MindVision相机

launch方式运行：

```bash
ros2 launch rmoss_entity_cam mindvision_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)