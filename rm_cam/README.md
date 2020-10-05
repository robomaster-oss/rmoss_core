# rm_cam模块

## 1.简介

rm_cam是rmoss_core 中的一个基础功能包，提供了usb相机ROS节点和虚拟相机ROS节点相关功能，同时，还支持二次开发，支持自定义相机扩展，加速开发。主要实现功能如下：

- **usb相机ROS节点**
- **基于图片的模拟相机ROS节点**
- **基于视频的模拟相机ROS节点**

为了提高扩展能力，该模块将 **相机操作** 与 **相机ROS节点** 进行解偶，通过 **相机接口** 实现，所以，对于不同的工业相机，有不同的驱动，通过该 **相机接口** ，可以无需关心ROS部分，快速实现 **相机ROS节点** 。详细介绍参考二次开发部分。

## 2.文件说明

主要文件：

|          文件           |                功能描述                |
| :---------------------: | :------------------------------------: |
|   cam_dev_interface.h   |          定义通用相机设备接口          |
|    usb_cam_dev.h/cpp    |            usb相机设备实现             |
| sim_cam_image_dev.h/cpp |          图片模拟相机设备实现          |
| sim_cam_video_dev.h/cpp |          视频模拟相机设备实现          |
|    camera_task.h/cpp    | ros顶层模块，实现通用相机ROS节点封装。 |

node文件:

|          文件           |      功能描述       |
| :---------------------: | :-----------------: |
|    task_usb_cam.cpp     |   usb相机ROS节点    |
| task_sim_cam_image_.cpp | 图片模拟相机ROS节点 |
| task_sim_cam_video.cpp  | 视频模拟相机ROS节点 |

## 3.快速使用

#### usb相机：

运行：

```bash
ros2 launch rm_cam usb_cam.launch.py  #使用/dev/video0
```

launch文件说明：

* 略

#### 图片模拟相机：

运行：

```bash
ros2 launch rm_cam sim_cam_image.launch.py  #使用默认图片test.png
```

launch文件说明：

* 略

## 4.二次开发

### a.cam_dev_interface接口

* 整个相机模块通过cam_dev_interface接口，该接口定义了设备规范，实现了模块的可扩展性

以下为接口API，其中前三个为必须实现的，其他为可选。

```c++
//required.
bool isOpened();//设备是否打开
int capImg(cv::Mat &img);//获取图像，成功获取则返回0
//option.参数设置和获取接口，默认实现返回false
bool setParameter(CamParameter parameter,int value);
bool getParameter(CamParameter parameter,int& value);
```

__相机参数（CamParameter）说明：__

```c++
ResolutionWidth;//分辨率宽
ResolutionHeight;//分辨率高
Exposure;//曝光，值为0代表自动曝光
Brighthness;//亮度
WhiteBalance;//白平衡，值为0代表自动白平衡
Gain;//增益
Gamma;//伽马值
Contrast;//对比度
Saturation;//饱和度
Hue;//色调
Fps;//帧率
```

- 不同相机值参数的取值范围不同，需要根据具体相机型号进行参数设置。

### b.整体结构模型

以task_usb_cam为例，模块包含两个组件，usb_cam_dev和camera_task，采用接口设计，定义了相机设备接口（cam_dev_interface），通过接口连接两个组件：

- usb_cam_dev：相机设备组件，实现了cam_dev_interface接口，负责获取相机图像，与ROS无关。
- camera_task：ROS节点组件，一个简单的通用相机ROS节点封装，负责ROS图像数据传输。

> 自定义rm_cam dev
>
> * 只需要实现cam_dev_interface接口，通过接入camera_task通用相机ROS节点，就能实现ROS图像发布节点功能。
> * usb_cam_dev，sim_cam_image_dev，sim_cam_video_dev均实现了cam_dev_interface接口。

## 5.维护者及开源许可证

* gezp zhenpeng.ge@qq.com

* rm_cam is provided under MIT.

