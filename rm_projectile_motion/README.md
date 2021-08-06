# rm_projectile_motion模块

## 1.简介

rm_projectile_motion是rmoss_core中的一个基础功能包，对子弹在飞行弹道进行建模，根据目标位置，计算出云台所需要的转角。

## 2.文件说明

主要文件：

|             文件              |                  功能描述                   |
| :---------------------------: | :-----------------------------------------: |
| projectile_model_interface.hpp | 抛物弹道逆求解接口，给定目标位置，计算所需仰角 |
| iteration_projectile_model.hpp/cpp | 弹道逆求解的数值迭代模型，给定子弹飞行正运动学，逆求解仰角。 |
| gravity_projectile_model.hpp/cpp | 考虑重力因素的弹道逆求解模型 |
| gaf_projectile_model.hpp/cpp | 考虑重力和空气摩檫阻力因素的弹道逆求解模型（不稳定） |
| gimbal_transform_tool.hpp/cpp | 云台转角工具：根据3D坐标点，利用弹道模型计算pitch,yaw角度。 |

## 3.使用方法

该包不支持ROS节点单独运行，只能通过库依赖的方式被调用。

```c++
//创建弹道模型，GravityProjectileModel，参数为25，代表子弹速度为25
auto projectile_model = std::make_shared<rm_projectile_motion::GravityProjectileModel>(25);
//创建gimbal_transform_tool，并设置弹道模型.
projectile_tansform_tool_ = std::make_shared<rm_projectile_motion::GimbalTransformTool>();
projectile_tansform_tool_->setProjectileModel(projectile_model);
//projectile_tansform_tool_->setProjectileModel(NULL);//直线模型。
//求解例子
cv::Point3f position(6,2,2);
float pitch,yaw;
projectile_tansform_tool_->transform(position,pitch,yaw)
```

* position坐标系为右手坐标系，枪口为x方向

**迭代弹道模型数学原理** : [rmoss_tutorials/projectile_motion_iteration](https://robomaster-oss.github.io/rmoss_tutorials/#/rmoss_core/rm_projectile_motion/projectile_motion_iteration)

## 4.维护者及开源许可证

- Zhenpeng Ge,  zhenpeng.ge@qq.com

* rm_projectile_motion is provided under Apache License 2.0