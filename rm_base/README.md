# rm_base模块

## 1.简介

rm_base是rmoss_core 中的一个基础功能包，提供了与机器人底层STM32通信的相关功能，是计算机与机器人之间的数据桥梁，实现了数据收发功能。

* 通过rm_base给机器人底层STM32发生控制数据，来控制机器人行动。
* 接收机器人的底层STM32的数据，并发布成ROS topic，供程序使用。

同时，还支持二次开发，其中的数据包封装模块，串口驱动模块等模块，可以直接使用，开发自己的机器人通信模块，加速开发。

主要文件：

* `transporter_interface.hpp` : 定义通用数据传输设备接口
* `uart_transporter.hpp/cpp` : 串口设备通用驱动（实现了`TransporterInterface`接口）
* `fixed_packet.hpp` : 固定长度数据包，封装了数据拆包，解包，校验等功能（模板类，支持不同长度的包）。
* `fixed_packet_tool.hpp` : 固定数据包收发工具（模板类，与`fixed_packet.hpp` 有关）
* `robot_base_example.hpp/cpp` : ROS顶层模块，ROS与STM32通信示例。

## 2.快速使用

提供了一个开发样例，可以快速验证，同时也为二次开发提供了参考。

__通信协议示例：__

* [protocol_example](doc/protocol_example .md)

__修改配置文件：__

需要准备一个usb串口模块

__运行节点：__

```bash 
 ros2 run rm_base robot_base_example
```

## 3.二次开发

* `fixed_packet.hpp/cpp`：提供了数据包抽象类，封装了数据包与字节流之间的相互转化操作。
* `fixed_packet_tool.hpp/cpp` : 在依赖通信的设备的情况下，进一步简化通信过程，可直接直接发送与接收数据包，封装底层操作。

### 3.1 FixedPacket模块类

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

__创建数据包：__

* 新建对象->加载数据-> 打包

```c++
FixedPacket16 packet;  // 新建对象
float angle=10;
// 显式装载数据（建议显式）
packet.load_data<float>(angle,3);  // 一个参数为数据，第二个数据为数据位置
// 隐式装载
packet.load_data(angle,3);
packet.pack();  // 打包数据
```

__解析数据包：__

* 新建对象 -> 校验buffer-> 复制buffer存入包中-> 取出数据


```c++
/*******自定义解析数据************/
FixedPacket16 packet;//新建对象
//check and copy recv_buffer
packet.check(recv_buffer, recv_len);
packet.copy_from(recv_buffer);
float angle=0;
packet.unload_data(angle,3);//取出数据（隐式，建议隐式）
```

* 发送与接收FixedPacket.

### 3.2 FixedPacketTool模板类

* 利用FixedPacketTool简化了数据传输流程，不需要考虑底层字节数据传输细节。

```c++
//需要依赖通信设备，假设已经获得正常工作的通信设备。
TransporterInterface::SharedPtr transporter;
//实例化数据包工具，初始化的时候，传入通信设备指针
FixedPacket16Tool::SharedPtr packet_tool_;
packet_tool_=std::make_shared<FixedPacket16Tool>(transporter);
```

**利用FixedPacketTool发送数据** 

```c++
#数据包
FixedPacket16 packet;
packet.load_data<unsigned char(protocol_example::GimbalAngleControl,1);
packet.load_data<unsigned char>(0x00,2);
packet.load_data<float>(info->pitch_angle,3);
packet.load_data<float>(info->yaw_angle,7);
packet.pack();
#发送
packet_tool_->send_packet(packet);
```

**利用FixedPacketTool接收数据** 

```c++
FixedPacket16 packet;
//  recv_packet()为堵塞函数，并已经包含校验等操作。
while(packet_tool_->recv_packet(packet)){
	unsigned char cmd;
	packet.unload_data(cmd,1);
｝
```

### 3.3 TransporterInterface接口

```c++
virtual bool open() = 0;
virtual void close() = 0;
virtual bool is_open() = 0;
virtual int read(unsigned char * buffer, int len) = 0;  // 接收数据
virtual int write(const unsigned char * buffer, int len) = 0;  // 发送数据
```

* 可参考`UartTransporter` 实现，未来考虑实现`UdpTransporter`

