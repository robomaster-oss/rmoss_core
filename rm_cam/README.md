# rm_cam模块

## 1.简介

rm_cam是rmoss_core 中的一个基础功能包，提供了usb相机ROS节点和虚拟相机ROS节点相关功能，同时，还支持二次开发，支持自定义相机扩展，加速开发。主要实现功能如下：

- **usb相机ROS节点**
- **基于图片的模拟相机ROS节点**
- **基于视频的模拟相机ROS节点**

为了提高扩展能力，该模块将 **相机操作** 与 **相机ROS节点** 进行解偶，通过 **相机接口** 实现，所以，对于不同的工业相机，有不同的驱动，通过该 **相机接口** ，可以无需关心ROS部分，快速实现 **相机ROS节点** 。详细介绍参考二次开发部分。

## 2.文件说明

主要文件：

|         文件          |                   功能描述                   |
| :-------------------: | :------------------------------------------: |
|  cam_dev_interface.h  |             定义通用相机设备接口             |
|   usb_cam_dev.h/cpp   |               usb相机设备实现                |
| virtual_cam_dev.h/cpp | 虚拟相机设备实现（基于图片和基于视频的方式） |
|   camera_task.h/cpp   |    ROS顶层模块，实现通用相机ROS节点封装。    |

node文件:

|            文件            |         功能描述          |
| :------------------------: | :-----------------------: |
|      usb_cam_node.cpp      |      usb相机ROS节点       |
| virtual_image_cam_node.cpp | 基于图片的虚拟相机ROS节点 |
| virtual_video_cam_node.cpp | 基于视频的虚拟相机ROS节点 |

## 3.快速使用

### 3.1 usb相机：

launch方式运行：

```bash
ros2 launch rm_cam usb_cam.launch.py  #使用默认/dev/video0
```

参数说明（usb_cam需要5个参数，launch文件中）

```python
            parameters=[
                {'cam_topic_name': 'usb_cam/image_raw'},
                {'usb_cam_path': '/dev/video0'},
                {'cam_width': 1280},
                {'cam_height': 720},
                {'cam_fps': 20}
            ],
```

### 3.2 基于图片的虚拟相机：

launch方式运行：

```bash
ros2 launch rm_cam virtual_image_cam.launch.py  #使用默认图片res/test.png
```

参数说明（virtual_image_cam需要3个参数，launch文件中）

```python
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                {'image_path': image_path},
                {'cam_fps': 30}
            ]
```

### 3.3 基于视频的虚拟相机

```bash
ros2 run rm_cam virtual_video_cam --ros-args -p "cam_topic_name:=virtual_cam/image_raw" -p "image_path:=/home/ubuntu/test.avi"
```

* virtual_video_cam需要2个参数

## 4.二次开发

### 4.1 cam_dev_interface接口

* 整个相机模块通过cam_dev_interface接口，该接口定义了设备规范，实现了模块的可扩展性

以下为接口API，其中前三个为必须实现的，其他为可选。

```c++
//required.
bool isOpened();//设备是否打开
bool capImg(cv::Mat &img);//获取图像
//option.参数设置和获取接口，默认实现返回false
bool setParameter(CamParameter parameter,int value);
bool getParameter(CamParameter parameter,int& value);
```

__相机参数（CamParameter）说明：__

```c++
CamParameter::resolution_width;//分辨率宽
CamParameter::resolution_height;//分辨率高
CamParameter::auto_exposure;//自动曝光,1代表自动曝光设置，0代表手动曝光设置
CamParameter::exposure;//曝光值
CamParameter::brighthness;//亮度
CamParameter::auto_white_balance;//自动白平衡，1代表自动白平衡设置，0代表手动白平衡设置
CamParameter::white_balance;//白平衡
CamParameter::gain;//增益
CamParameter::gamma;//伽马值
CamParameter::contrast;//对比度
CamParameter::saturation;//饱和度
CamParameter::hue;//色调
CamParameter::fps;//帧率
```

- 不同相机值参数的取值范围不同，需要根据具体相机型号进行参数设置，仅支持整型设置。

### 4.2  整体结构模型

以task_usb_cam为例，模块包含两个组件，usb_cam_dev和camera_task，采用接口设计，定义了相机设备接口（cam_dev_interface），通过接口连接两个组件：

- usb_cam_dev：相机设备组件，实现了cam_dev_interface接口，负责获取相机图像，与ROS无关。
- camera_task：ROS节点组件，一个简单的通用相机ROS节点封装，负责ROS图像数据传输。

> 自定义rm_cam dev
>
> * 只需要实现cam_dev_interface接口，通过接入camera_task通用相机ROS节点，就能实现ROS图像发布节点功能。
> * usb_cam_dev，virtual_cam_dev均实现了cam_dev_interface接口。

## 5.维护者及开源许可证

maintainer：Zhenpeng Ge,  zhenpeng.ge@qq.com

rm_cam is provided under Apache License 2.0.

