# rmoss_base模块

## 简介

rmoss_base是rmoss_core 中的一个基础功能包，提供了与机器人底层STM32通信的相关功能，是计算机与机器人之间的数据桥梁，实现了数据收发功能。

* 通过rmoss_base给机器人底层STM32发生控制数据，来控制机器人行动。
* 接收机器人的底层STM32的数据，并发布成ROS topic，供程序使用。

同时，还支持二次开发，其中的数据包封装模块，串口驱动模块等模块，可以直接使用，开发自己的机器人通信模块，加速开发。

主要文件：

* `transporter_interface.hpp` : 定义通用数据传输设备接口
* `uart_transporter.hpp/cpp` : 串口设备通用驱动（实现了`TransporterInterface`接口）
* `fixed_packet.hpp` : 固定长度数据包，封装了数据加载，卸载功能，提供了数据包与字节流之间的相互转化操作（模板类，支持不同长度的包）。
* `fixed_packet_tool.hpp` : 固定数据包收发工具，可直接直接发送与接收数据包，封装底层操作（模板类，与`fixed_packet.hpp` 有关）
* `simple_robot_base_node.hpp/cpp` : ROS顶层模块`SimpleRobotBaseNode`，ROS与STM32通信示例。
* `simple_robot_base_main.cpp` : `SimpleRobotBaseNode`节点的main()入口。

## 快速使用

提供了一个开发样例，可以快速验证，同时也为二次开发提供了参考。

运行节点，需要修改配置文件，并准备一个usb串口模块：

```bash 
 ros2 run rmoss_base simple_robot_base
```

* `SimpleRobotBaseNode`节点已经注册为`rclcpp component`, 支持[ROS Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)方式启动。

## 二次开发

### FixedPacket模块类

```c++
// FixedPacket模块类
FixedPacket<16> packet;  // 16 Byte定长数据包
FixedPacket<32> packet;  // 32 Byte定长数据包
// 预定义数据包
FixedPacket packet;    // 默认定长数据包，长度为16
FixedPacket16 packet； // 等价于FixedPacket<16>
FixedPacket32 packet； // 等价于FixedPacket<32>
FixedPacket64 packet； // 等价于FixedPacket<64>
```

数据包数据布局 （以16 Byte定长数据包为例）

| 数据头字节（0xff） |   数据字节    |  校验字节   | 数据尾字节（0x0d） |
| :----------------: | :-----------: | :---------: | :----------------: |
|     0（1Byte）     | 1-13 (13Byte) | 14（1Byte） |    15（1Byte）     |

数据包操作

```c++
FixedPacket<16> packet;  // 新建对象
/*******自定义加载数据************/
float angle=10;
packet.load_data<float>(angle,3);  // 一个参数为数据，第二个数据为数据位置（建议显式）
// 隐式装载
packet.load_data(angle,3);
/*******自定义卸载数据************/
float angle2=0;
packet.unload_data(angle,3);//取出数据（隐式，建议隐式）
/**********其他操作**************/
packet.clear();  // 清零数据包中的数据字节和校验字节
packet.copy_from();  // 拷贝并覆盖数据包中中的buffer
packet.buffer();  // 获取数据包中的buffer (const修饰，只读)
```

* 其中只有数据位字节才能使用`load_data()`和`unload_data()`操作，校验字节可通过`set_check_byte()`进行设置。

### FixedPacketTool模板类

利用FixedPacketTool简化了数据传输流程，不需要考虑底层字节数据传输细节。

```c++
// 需要依赖通信设备，假设已经获得正常工作的通信设备。
TransporterInterface::SharedPtr transporter;
// 实例化数据包工具，初始化的时候，传入通信设备指针
FixedPacket16Tool::SharedPtr packet_tool_;
packet_tool_=std::make_shared<FixedPacketTool<16>>(transporter);
```

利用FixedPacketTool发送数据

```c++
#数据包
FixedPacket<16> packet;
packet.load_data<unsigned char(protocol_example::GimbalAngleControl,1);
packet.load_data<unsigned char>(0x00,2);
packet.load_data<float>(info->pitch_angle,3);
packet.load_data<float>(info->yaw_angle,7);
#发送
packet_tool_->send_packet(packet);
```

利用FixedPacketTool接收数据

```c++
FixedPacket<16> packet;
// recv_packet()为堵塞函数，并已经包含校验等操作。
while(packet_tool_->recv_packet(packet)){
	unsigned char cmd;
	packet.unload_data(cmd,1);
｝
```

### TransporterInterface接口

```c++
virtual bool open() = 0;
virtual void close() = 0;
virtual bool is_open() = 0;
virtual int read(void * buffer, size_t len) = 0;  // 接收数据
virtual int write(const void * buffer, size_t len) = 0;  // 发送数据
virtual std::string error_message() = 0;  // 当open()返回false时，获取error message。
```

* 可参考`UartTransporter` 和`UdpTransporter` 的实现。
