![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

* 更多内容详见[https://robomaster-oss.github.io](https://robomaster-oss.github.io)


# rmoss_core

**正在开发中，部分功能不稳定。。。**

rmoss_core是RoboMaster OSS中的基础项目，为RoboMaster提供通用基础功能模块包，如相机模块，弹道运动模块等。

* 目前仅支持ROS2 foxy版本

## 1.主要模块

|          模块           |                          功能说明                           |
| :---------------------: | :---------------------------------------------------------: |
| rm_interfaces（不稳定） | RM相关的ROS interface包，包含相关msg，srv，action定义文件。 |
|         rm_base         |  基本通信工具包，包含PC与嵌入式系统（stm32）通信相关工具。  |
|        rm_common        |        公共工具包，包括调试，图像处理等公共基础工具         |
|         rm_cam          |                         相机工具包                          |
|         rm_task         |         通用图像处理任务包（被当作一个依赖库使用）          |
|  rm_projectile_motion   |                     通用弹道模型工具包                      |

## 2.维护者及开源许可证

* gezp zhenpeng.ge@qq.com

* rmoss_core is provided under MIT.
