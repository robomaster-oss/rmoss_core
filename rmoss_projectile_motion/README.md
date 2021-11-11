# rmoss_projectile_motion模块

## 简介

rmoss_projectile_motion是rmoss_core中的一个基础功能包，对子弹在飞行弹道进行建模，根据目标位置，计算出云台所需要的转角。

主要文件：

* `iterative_projectile_solver.hpp/cpp` : 基于数值迭代的弹道逆运动求解器，即给定子弹飞行正运动学方程，可根据给定目标位置，计算所需仰角。
* `gravity_projectile_solver.hpp/cpp` : 考虑重力因素的弹道逆运动求解器
* `gaf_projectile_solver.hpp/cpp` : 考虑重力和空气摩擦阻力因素的弹道逆运动求解器（不稳定）
* `gimbal_transform_tool.hpp/cpp` : 云台转角计算工具，需要传入求解器，利用弹道模型计算pitch, yaw角度

## 使用方法

该包不支持ROS节点单独运行，只能通过库依赖的方式被调用。

```c++
//创建基于重力弹道模型的求解器，GravityProjectileSolver，参数为25，代表子弹速度为25
auto solver = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(25);
//创建gimbal_transform_tool，传入求解器
projectile_tansformoss_tool = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(solver);
//求解例子
Eigen::Vector3d position(6,2,2);
double pitch,yaw;
projectile_tansformoss_tool_->solve(position,pitch,yaw);
```

* position坐标系为右手坐标系，枪口为x方向

**迭代弹道模型数学原理** : [rmoss_tutorials/projectile_motion_iteration](https://robomaster-oss.github.io/rmoss_tutorials/#/rmoss_core/rmoss_projectile_motion/projectile_motion_iteration)
