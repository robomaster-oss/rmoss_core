# rm_task模块

## 1.简介

rm_task是rmoss_core中的一个基础功能包，为用户开发图像处理相关的任务提供相关支持，加快开发速度。主要实现了以下功能：

* 图像任务封装，使得用户不需要考虑ROS2图像订阅等步骤，只需要关心图像处理即可。

## 2.文件说明

主要文件：

|             文件              |                  功能描述                   |
| :---------------------------: | :-----------------------------------------: |
| task_image_proc.h/cpp | 图像任务基类，封装了ROS2图像订阅等相关操作。 |

example：（使用例子）

|              文件              |                    功能描述                     |
| :----------------------------: | :---------------------------------------------: |
|     task_show_image.h/cpp      | 开发样例，基于task_image_proc，实现显示图片任务 |
| nodes/task_show_image_node.cpp |         ROS2节点，运行task_show_image。         |

## 3.快速使用

__step1:运行图像发布节点__

首先，需要启动图像发布节点，实现图像采集功能，这里利用usb相机:

运行：

```bash
ros2 launch rm_cam sim_cam_image.launch.py 
```

测试：

```bash
ros2 topic list #查看所有节点，检查图像节点是否发布
ros2 run rqt_image_view rqt_image_view#ros调试工具，图形化显示图像工具
```

__step2:运行图像处理节点__

运行：

```bash
ros2 launch rm_task task_show_image.launch.py 
```

* 图像将会实时显示（类似rqt_image_view的功能）

## 4.维护者及开源许可证

- gezp zhenpeng.ge@qq.com

* rm_task is provided under MIT