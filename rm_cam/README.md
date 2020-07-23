# robot_cam模块

## 1.简介

robot_cam是RoboMasterOS 中的一个基础功能包，提供了usb相机ROS节点和虚拟相机ROS节点相关功能，同时，还支持二次开发，支持自定义相机扩展，加速开发。主要实现功能如下：

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
| robot_cam_example.h/cpp | ros顶层模块，实现通用相机ROS节点封装。 |

node文件:

|          文件          |      功能描述       |
| :--------------------: | :-----------------: |
|    usb_cam_node.cpp    |   usb相机ROS节点    |
| sim_cam_image_node.cpp | 图片模拟相机ROS节点 |
| sim_cam_video_node.cpp | 视频模拟相机ROS节点 |

## 3.快速使用

__配置文件通用参数__：/res目录下

```yaml
cam_topic_name: "front_camera"
cam_fps: 30
```

- cam_topic_name：ROS发布图片topic节点名字。
- cam_fps：可以控制图片发布（topic）帧率。

### a.usb相机：

修改配置文件：res/usb_cam_config.yaml

```yaml
#video_path，可自行修改
usb_cam_path : "/dev/video0"
```

运行说明：

```bash
rosrun robot_cam usb_cam_node [optional config_path] #默认使用usb_cam_config.yaml
```

运行：

```bash
#master
roscore
#cam node
rosrun robot_cam usb_cam_node
```

### b.图片模拟相机：

运行说明：

```yaml
#需要两个参数
rosrun robot_cam sim_cam_image_node [config_path] [img_path]
```

运行：(采用launch方式)

```bash
roslaunch robot_cam sim_cam_image.launch
```

### c.视频模拟相机：

运行说明：

```yaml
#需要两个参数
rosrun robot_cam sim_cam_video_node [config_path] [video_path]
```

* 无需设置cam_fps，相机的FPS值将自动设为视频的帧率，无法修改。

运行：(采用launch方式)

```bash
roslaunch robot_cam sim_cam_video.launch
```

* 无法运行，需要设置launch文件中的视频路径参数才能运行

## 4.二次开发

### a.cam_dev_interface接口

* 整个相机模块通过cam_dev_interface接口，该接口定义了设备规范，实现了模块的可扩展性

以下为接口API，其中前三个为必须实现的，其他为可选。

```c++
//required.
int init(std::string config_path);//初始化接口，从配置文件初始化
bool isOpened();//设备是否打开
int capImg(cv::Mat &img);//获取图像，成功获取则返回0
//option.参数设置和获取接口，默认实现返回false
bool setParameter(CamParameter parameter,int value);
bool getParameter(CamParameter parameter,int& value);
bool setParameters(std::string config_path);
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

robot_cam模块包含两个组件，采用接口设计，定义了相机设备接口（cam_dev_interface），通过接口连接两个组件：

- robot_cam_dev：相机设备组件，实现了cam_dev_interface接口，负责获取相机图像，与ROS无关。
- robot_cam_example：ROS节点组件，一个简单的通用相机ROS节点封装，负责ROS图像数据传输。

![](doc/imgs/robot_cam_node.png)

### c.自定义robot_cam node

robot_cam node的实现只需要robot_cam_dev，robot_cam_example两个模块。__usb_cam_node为例__：在main函数中

```c++
//robot_cam_dev
robot_cam::UsbCamDev cam_intercace;
cam_intercace.init(config_path);
//robot_cam_example,需要接入cam_intercace
robot_cam::RobotCamExample cam_example;
cam_example.init(config_path,&cam_intercace);
```

### d.自定义robot_cam dev

只需要实现cam_dev_interface接口，通过接入robot_cam_example通用相机ROS节点，就能实现ROS图像发布节点功能。

- usb_cam_dev，sim_cam_image_dev，sim_cam_video_dev均实现了cam_dev_interface接口。

### e.自定义robot_cam_example

如果需要自定义ROS顶层，可以实现自己的ROS相机顶层，具体实现参考robo_cam_example.h/cpp。这里不再具体说明。

## 5.维护者及开源许可证

* gezp 1350824033@qq.com

robot_cam is provided under GPL-v3.

