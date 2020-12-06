![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

> [RoboMaster竞赛](https://www.robomaster.com/)，全称为`全国大学生机器人大赛RoboMaster机甲大师赛` 。
>
> - 全国大学生机器人[RoboMaster](https://www.robomaster.com/)大赛，是一个涉及“机器视觉”、“嵌入式系统设计”、“机械控制”、“人机交互”等众多机器人相关技术学科的机器人比赛。
> - 在RoboMaster 2019赛季中，参赛队伍需自主研发不同种类和功能的机器人，在指定的比赛场地内进行战术对抗，通过操控机器人发射弹丸攻击敌方机器人和基地。每局比赛7分钟，比赛结束时，基地剩余血量高的一方获得比赛胜利。
>
> 更多详情参考官网：[www.robomaster.com](https://www.robomaster.com/)

# rmoss_core

rmoss_core是RoboMaster OSS中的基础项目，为RoboMaster提供通用基础功能模块包，如相机模块，弹道运动模块等。

## 1.主要模块

|          模块          |                          功能说明                           |
| :--------------------: | :---------------------------------------------------------: |
|      `rm_common`       |       公共工具包，包括调试，图像处理等公共基础工具。        |
|       `rm_base`        |  基本通信工具包，包含PC与嵌入式系统（stm32）通信相关工具。  |
|        `rm_cam`        |     相机工具包，实现usb相机驱动，以及图片视频虚拟相机。     |
|       `rm_task`        |         任务相关工具，提供了一个图像相关任务基类。          |
| `rm_projectile_motion` | 通用弹道模型工具包，可以修正子弹飞行过程中重力因素的影响。  |

## 2.使用说明

* 目前仅支持`ROS2 foxy`版本
* 环境依赖：
  *  [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces) : ROS2 interfaces (.msg, .srv, .action) used in the RoboMaster OSS Projects

环境配置

```bash
#cd ros2 workspaces src
git clone https://github.com/robomaster-oss/rmoss_interfaces.git
git clone https://github.com/robomaster-oss/rmoss_core.git
#cd ros2 workspaces
colcon build
```

* 相关功能包使用详见相应package的README.md

## 3.维护者及开源许可证

Maintainer : Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_core is provided under MIT