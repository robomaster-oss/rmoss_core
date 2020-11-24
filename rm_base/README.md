# rm_base模块

## 1.简介

rm_base是RoboMasterOS 中的一个基础功能包，提供了与机器人底层STM32通信的相关功能，是计算机与机器人之间的数据桥梁，实现了数据收发功能。

* 通过rm_base给机器人底层STM32发生控制数据，来控制机器人行动，
* 接收机器人的底层STM32的数据，并发布成ROS topic，供程序使用。

同时，还支持二次开发，其中的数据包封装模块，串口驱动模块等模块，可以直接使用，开发自己的机器人通信模块，加速开发。

## 2.文件说明

主要文件：

|          文件           |                      功能描述                      |
| :---------------------: | :------------------------------------------------: |
|  comm_dev_interface.h   |              定义通用数据传输设备接口              |
|    serialport.h/cpp     |                  串口设备通用驱动                  |
|  serialport__dev.h/cpp  |    串口传输设备（实现了comm_dev_interface接口）    |
|   fixed_packet.h/cpp    | 固定长度数据包，封装了数据拆包，解包，校验等功能。 |
| fixed_packet_tool.h/cpp |                 固定数据包收发工具                 |

exampe文件：(提供了一个开发样例)

|               文件                |       功能描述        |
| :-------------------------------: | :-------------------: |
|        protocol_example.h         |  通信协议命令宏定义   |
|       rm_base_example.h/cpp       | 机器人通信ROS顶层封装 |
| nodes/robot_base_example_node.cpp | 创建机器人通信ROS节点 |

## 3.快速使用

提供了一个开发样例，可以快速验证，同时也为二次开发提供了参考。

__样例协议：__

* [protocol_example](doc/protocol_example .md)

__修改配置文件：__

需要准备一个usb串口模块

__运行节点：__

```bash 
 ros2 run rm_base robot_base_example
```

## 4.二次开发

* fixed_packet.h/cpp：数据包封装，详细使用参考文档[fixed_packet.md](doc/fixed_packet.md)
* fixed_packet_tool.h/cpp : 固定数据包收发工具，详细使用参考文档[fixed_packet.md](doc/fixed_packet.md)

## 5.维护者及开源许可证

- Zhenpeng Ge,  zhenpeng.ge@qq.com

* rm_base is provided under MIT.