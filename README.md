# rmoss_core

![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

* 更多内容详见[github.io](github.io)

**正在开发中，部分功能不稳定。。。**
rmoss_core是RoboMasterOSS中的一个基础项目，基于ROS2开发，采用了模块化设计方法，提供多个功能模块。

* 目前仅支持ROS2 foxy版本

## 1.主要模块

|         模块         |                  功能说明                  |
| :------------------: | :----------------------------------------: |
|       rm_msgs        |           包含相关msg，srv定义。           |
|       rm_base        | 基本通信工具包，包含步兵和哨兵的底层通信。 |
|       rm_tool        |    基本工具包，包括调试，图像处理等工具    |
|        rm_cam        |              机器人相机工具包              |
|       rm_task        | 通用图像处理任务包（被当作一个依赖库使用） |
| rm_projectile_motion |             通用弹道模型工具包             |

## 2.维护者及开源许可证

- zhenpeng.ge@qq.com

rmoss_core is provided under MIT.