# rmoss_util模块

## 1.简介

rmoss_util是rmoss_core 中的一个公共基础包，提供一些公共基础工具及功能。主要实现了以下几个功能：

* 提供一些调试工具（如在图像上绘制四边形，显示图片，调试开关等）
* 提供一些简单图像处理工具（如基于单目视觉的3D位置解算工具，数学公式函数实现等）

主要文件：

* `types.hpp` : 公共定义
* `debug.hpp/cpp` : 调试工具，相关调试工具
* `math.hpp/cpp` : 数学工具，提供一些图像处理或计算相关工具
* `mono_measure_tool.hpp/cpp` : 单目测量工具类，单目算法封装（PNP解算，相似三角形反投影等）

## 2.快速使用

#### debug模块

静态调试开关：控制是否显示调试信息

```c++
#include "rmoss_util/debug.hpp"
//获取调试开关状态，默认为false，
rmoss_util::get_debug();
//设置静态调试开关
rmoss_util::set_debug(true);
```

通用调试宏定义

```c++
//rmoss_DEBUG(text); text为一条语句 ，如果静态调试开关为false，括号里面的语句不会被执行。
rmoss_DEBUG(imshow("dst", dst));
rmoss_DEBUG(std::cout<<"data"<<std::endl);
```

图像调试函数

```c++
//在图像上绘制多边形
void draw_rotated_rect(cv::Mat &img,cv::RotatedRect r,cv::Scalar color=green);
void draw_4points(cv::Mat &img, cv::Point2f *point2fs,cv::Scalar color=green);
void draw_convex_hull(cv::Mat &img,std::vector<cv::Point2f> points,cv::Scalar color=green);
//使用
rmoss_DEBUG(rmoss_util::draw_rotated_rect(img,r));
```

#### math模块

提供相关图像计算工具

```c++
//两点构成直线的倾角，相对常规坐标系，（不同于图像坐标系）常规坐标系y轴方向为向上。
float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
//三角形的角度，第一个参数为顶点坐标
float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
```

#### mono_measure_tool模块

工具类，主要包括基于PNP，和相似三角形投影算法的单目算法封装，2d->3d点位置解算。

```c++
rmoss_util::MonoMeasureTool mono_location_tool_;
mono_location_tool_.set_camera_info(camera_intrinsic, camera_distortion);
mono_location_tool_.solve_pnp(detected_points, small_armor_points, target_postion);
```

## 3.维护者及开源许可证

Maintainer:

* wyx, 1418555317@qq.com

* Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_util is provided under Apache License 2.0.
