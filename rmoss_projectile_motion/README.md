# rmoss_projectile_motion模块

## 简介

rmoss_projectile_motion是rmoss_core中的一个基础功能包，对子弹在飞行过程的弹道进行建模，根据目标位置，计算出云台所需要的转角。

* `GimbalTransformTool` : 云台转角计算工具，给定目标点，利用弹道模型计算射击所需要云台的pitch, yaw角度，需要传入弹道逆运动求解器。

目前rmoss_projectile_motion提供了两种求解器：

* `GravityProjectileSolver` : 考虑重力的弹道逆运动求解器 （抛物型模型）
* `GafProjectileSolver` : 考虑重力和空气阻力的弹道逆运动求解器 （近似模型，不稳定）

`GravityProjectileSolver`和`GafProjectileSolver`都是基于数值迭代实现的求解器，定义不同的弹道运动模型，然后通过使用`IterativeProjectileTool`实现求解过程。


## 使用方法

该包不支持ROS节点单独运行，只能通过库依赖的方式被调用。

```c++
// 创建基于重力弹道模型的求解器，GravityProjectileSolver，参数代表子弹速度为25m/s
auto graviry_solver = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(25);

// 创建考虑重力和空气阻力的求解器，GafProjectileSolver，参数代表子弹速度为25m/s，空气阻力系数为0.1
auto gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(25, 0.1);

// 创建gimbal_transform_tool，传入graviry_solver求解器
auto projectile_tansformoss_tool =
  std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(graviry_solver);

// 求解例子
Eigen::Vector3d position(6, 2, 2);
double pitch, yaw;
projectile_tansformoss_tool->solve(position, pitch, yaw);

// 更换求解器
projectile_tansformoss_tool->set_projectile_solver(gaf_solver);

// 更改求解器的参数
graviry_solver->set_initial_vel(15);
gaf_solver->set_initial_vel(15);
gaf_solver->set_friction_coeff(0.2);
```

* position坐标系为右手坐标系，枪口为x方向

**迭代弹道模型数学原理** : [rmoss_tutorials/projectile_motion_iteration](https://robomaster-oss.github.io/rmoss_tutorials/#/rmoss_core/rmoss_projectile_motion/projectile_motion_iteration)
