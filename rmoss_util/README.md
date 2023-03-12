# rmoss_util模块

## 简介

rmoss_util是rmoss_core 中的一个公共基础包，提供一些公共基础工具及功能。主要实现了以下几个功能：

* 提供一些调试工具
* 提供一些简单图像处理工具

主要文件：
* `debug.hpp/cpp` : 调试工具
* `task_manager.hpp/cpp` : 任务管理封装，向外提供获取任务状态和控制任务服务。
* `image_utils.hpp/cpp`（不建议使用，开发中） : 图像工具，提供一些图像处理或计算相关工具。
* `time_utils.hpp/cpp`（不建议使用，开发中） : 时间工具，用于测量运行时间。
* `mono_measure_tool.hpp/cpp`（不建议使用，开发中） : 单目测量工具类，单目算法封装（PNP解算，相似三角形反投影等）
* `url_resolver.hpp/cpp` : URL 解析器，用于解析类似 camera_info_manager 的 URL，便于灵活路径管理

## 快速使用

### debug模块

静态调试开关：控制是否显示调试信息

```c++
#include "rmoss_util/debug.hpp"
//获取调试开关状态，默认为false，
rmoss_util::get_debug();
//设置静态调试开关
rmoss_util::set_debug(true);
```

通用调试宏使用

```c++
// RMOSS_DEBUG(text); text为一条语句 ，如果静态调试开关为false，括号里面的语句不会被执行。
RMOSS_DEBUG(imshow("dst", dst));
// 用于输出调试信息
RMOSS_DEBUG(std::cout<<"data"<<std::endl);
// 在图像上绘制多边形，用于调试图像
RMOSS_DEBUG(rmoss_util::draw_rotated_rect(img,r));
```

### TaskManager模块

使用样例
```c++
// task manager
auto get_task_status_cb = [&]() {
    return rmoss_util::TaskStatus::Running;
  };
auto control_task_cb = [&](rmoss_util::TaskCmd cmd) {
    if (cmd == rmoss_util::TaskCmd::Start) {
      // do something
    } else if (cmd == rmoss_util::TaskCmd::Stop) {
      // do something
    } else {
      return false;
    }
    return true;
  };
auto task_manager_ = std::make_shared<rmoss_util::TaskManager>(node_, get_task_status_cb, control_task_cb);
```
* 获取任务状态的service名字为：`<node_name>/get_task_status`
* 控制任务的service名字为：`<node_name>/control_task`

### image_utils模块

简单画图

```c++
//
void draw_rotated_rect(cv::Mat &img,cv::RotatedRect r,cv::Scalar color=green);
void draw_4points(cv::Mat &img, cv::Point2f *point2fs,cv::Scalar color=green);
void draw_convex_hull(cv::Mat &img,std::vector<cv::Point2f> points,cv::Scalar color=green);

```

数学计算

```c++
//两点构成直线的倾角，相对常规坐标系，（不同于图像坐标系）常规坐标系y轴方向为向上。
float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
//三角形的角度，第一个参数为顶点坐标
float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
```

### mono_measure_tool模块

工具类，主要包括基于PNP，和相似三角形投影算法的单目算法封装，2d->3d点位置解算。

```c++
rmoss_util::MonoMeasureTool mono_location_tool_;
mono_location_tool_.set_camera_info(camera_intrinsic, camera_distortion);
mono_location_tool_.solve_pnp(detected_points, small_armor_points, target_postion);
```

### URL 解析器

用于解析类似 camera_info_manager 的 URL

```c++
std::string url = "package://rmoss_util/test";
std::result = rmoss_util::URLResolver::getResolvedPath(url);  // result = "<rmoss_util的share路径>/test"

std::string url = "file:///test_dir/test_file";
std::result = rmoss_util::URLResolver::getResolvedPath(url);  // result = "/test_dir/test_file"
```