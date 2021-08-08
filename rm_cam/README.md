# rm_cam模块

## 1.简介

rm_cam是rmoss_core 中的一个基础功能包，提供了usb相机ROS节点和虚拟相机ROS节点相关功能，同时，还支持二次开发，支持自定义相机扩展，加速开发。主要实现功能如下：

- **usb相机ROS节点**
- **基于图片的虚拟相机ROS节点**
- **基于视频的虚拟相机ROS节点**

为了提高扩展能力，该模块将 **相机操作** 与 **相机ROS节点** 进行解偶，通过 **相机接口** 实现，所以，对于不同的工业相机，有不同的驱动，通过该 **相机接口** ，可以无需关心ROS部分，快速实现 **相机ROS节点** 。详细介绍参考二次开发部分。

文件说明：

* `cam_interface.hpp`：定义通用相机设备接口
* `usb_cam.hpp/cpp` : usb相机设备实现
* `virtual_cam.hpp/cpp`：虚拟相机设备实现，支持基于图片和基于视频两种方式。
* `cam_server.h/cpp` : ROS顶层模块，实现通用相机ROS节点封装，发布图像topic数据，支持获取相机参数服务。

## 2.快速使用

### 2.1 usb相机：

launch方式运行：

```bash
ros2 launch rm_cam usb_cam.launch.py  #使用默认/dev/video0
```

* 参数在yaml文件中（`config/cam_params.yaml`）

### 2.2 基于图片的虚拟相机：

launch方式运行：

```bash
ros2 launch rm_cam virtual_image_cam.launch.py  #使用默认图片resource/test.png
```

参数说明（在launch文件中设置）

```python
            parameters=[
                {'camera_name': 'front_camera'},
                {'image_path': image_path},
                {'cam_fps': 30},
                {'camera_matrix': [1552.7, 0.0, 640.0, 0.0, 1537.9, 360.0, 0.0, 0.0, 1.0]},
                {'camera_distortion': [0.0, 0.0, 0.0, 0.0, 0.0]},
            ],
```

* 至少需要一个参数`image_path`

采用`rqt_image_veiw` 查看图像`topic` 

```bash
ros2 run rqt_image_veiw rqt_image_veiw
```

### 2.3 基于视频的虚拟相机

```bash
ros2 run rm_cam virtual_video_cam --ros-args -p "video_path:=/home/ubuntu/test.avi"
```

* 至少需要一个参数`video_path`

## 3.二次开发

**cam_interface接口**

* 整个相机模块通过`CamInterface` 接口，该接口定义了设备规范，实现了模块的可扩展性，以下为接口API

```c++
//required.
bool open();  // 打开设备
void close();  // 关闭设备
bool is_open();  //检测设备是否打开
bool grab_image(cv::Mat &image);  //获取图像
//option.参数设置和获取接口，默认实现返回false
bool set_parameter(CamParamType type,int value);
bool get_parameter(CamParamType type,int& value);
```

__相机参数（CamParamType）说明：__

```c++
CamParamType::Width;  //分辨率宽
CamParamType::Height;  //分辨率高
CamParamType::AutoExposure;  //自动曝光,1代表自动曝光设置，0代表手动曝光设置
CamParamType::Exposure;  //曝光值
CamParamType::Brighthness;  //亮度
CamParamType::AutoWhiteBalance;  //自动白平衡，1代表自动白平衡设置，0代表手动白平衡设置
CamParamType::WhiteBalance;  //白平衡
CamParamType::Gain;  //增益
CamParamType::Gamma;  //伽马值
CamParamType::Contrast;  //对比度
CamParamType::Saturation;  //饱和度
CamParamType::Hue;  //色调
CamParamType::Fps;  //帧率
```

- 不同相机值参数的取值范围不同，需要根据具体相机型号进行参数设置，仅支持整型设置。
- 帧率参数Fps用于ROS节点发布图像，对于一般相机，可以忽略不用设置（某些特殊相机需要设置高帧率模式）。
- 参数设置应该在相机关闭下进行设置。

**ROS相机节点**

以usb_cam为例，在`main` 函数中创建相机设备与相机ROS服务。

```c++
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("usb_cam");
// 创建相机设备 (继承CamInterface接口)
auto cam_dev = std::make_shared<rm_cam::UsbCam>("/dev/video0");
// 创建相机ROS服务（需要传入一个相机设备）
auto cam_task = std::make_shared<rm_cam::CamServer>(node, cam_dev);
// run node until it's exited
rclcpp::spin(node);
// clean up
rclcpp::shutdown();
```

- `UsbCam`：相机设备组件，实现了cam_interface接口，负责获取相机图像，设置相机参数，与ROS无关。
- `CamServer` : 通用相机ROS节点组件，一个简单的通用相机ROS节点封装，负责ROS图像数据传输，支持获取相机参数服务。

> 自定义相机接口
>
> * 仿照UsbCam实现即可。

