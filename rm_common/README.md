# rm_tool模块

## 1.简介

rm_common是rmoss_core 中的一个工具包，加快开发速度。主要实现了以下几个功能：

* 提供时间相关工具封装
* 提供调试工具封装
* 提供一些图像调试工具（如在图像上绘制四边形，显示图片，调试开关等）
* 提供一些图像处理工具（如基于单目视觉的3D位置算工具等）

## 2.文件说明

主要文件：

|          文件           |                         功能描述                          |
| :---------------------: | :-------------------------------------------------------: |
|     time_tool.hpp/cpp   |                        时间工具类                         |
|     math.hpp/cpp        |          工具类，提供一些图像处理或计算相关工具           |
|      debug.hpp/cpp      |                   工具类，相关调试工具                    |
| mono_measure_tool.hpp/cpp | 工具类，一些单目算法封装（包含PNP解算，相似三角形投影等） |

## 3.快速使用


#### time_tool模块

* [time_tool_api文档](doc/time_tool_api.md)

#### math模块

提供相关图像计算工具

```c++
//两点构成直线的倾角，结果为度，相对常规坐标系，（不同于图像坐标系）常规坐标系y轴方向为向上。
static float calc2PointAngle(cv::Point2f point1,cv::Point2f point2);
//三角形的角度，第一个参数为顶点坐标,结果为度
static float calcTriangleInnerAngle(cv::Point2f vertexPoint,cv::Point2f point1,cv::Point2f point2);
```

#### debug模块

静态调试开关：控制是否显示调试信息

```c++
//默认为false，即调用该类的调试函数，均不会执行。
root_tool::ImageTool::is_debug=true
```

通用调试宏定义

```c++
//DEBUG_TOOL(text);text为一条语句
DEBUG_TOOL(imshow("dst", dst));
DEBUG_TOOL(cout<<"data"<<endl);
```

* 如果静态调试开关为false，括号里面的语句不会被执行。

图像调试宏定义

```c++
//api　type:0,blue;1,green;2,red.
static void drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type=0);
static void draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type=0);
static void drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type=0);
```

* 受静态调试开关控制

#### mono_measure_tool模块

工具类，主要包括基于PNP，和相似三角形投影算法的单目3d点位置解算。

* 详见文档[mono_measure_tool.md](doc/mono_measure_tool.md)

## 4.维护者及开源许可证

* wyx 1418555317@qq.com
* gezp zhenpeng.ge@qq.com　

* rm_tool is provided under MIT