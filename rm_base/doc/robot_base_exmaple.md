# robot_base_example详细说明

### 1.数据包发送

* 基于ROS Topic开发，当收到gimbal控制Topic时，将topic中的消息转化为FixedPacket，再使用send_packet()函数发送即可。

```c++
void RobotBaseExample::gimbalCallback(const rm_msgs::msg::GimbalControl::SharedPtr msg)
{
    FixedPacket packet;
    packet.load_data<unsigned char>(protocol_example::GimbalAngleControl, 1);
    packet.load_data<unsigned char>(0x00, 2);
    packet.load_data<float>(msg->pitch, 3);
    packet.load_data<float>(msg->yaw, 7);
    packet.pack();
    packet_tool_->send_packet(packet);
    //delay for data send.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

```

### 2.数据包接收

* 采用轮询接收方式，这里创建了了一个单独的新线程去监听设备，即mcuListenThread()函数。
* 在while中使用recv_packet()函数进行包接收，然后进行包中的数据处理。

```c++
void RobotBaseExample::mcuListenThread()
{
    FixedPacket packet;
    while (rclcpp::ok()){
        if (packet_tool_->recv_packet(packet)){
            //the packet have already unpacked.
            unsigned char cmd;
            packet.unload_data(cmd, 1);
            if (cmd == (unsigned char)protocol_example::ChangeMode){
                unsigned char mode = 0;
                packet.unload_data(mode, 2);
                if (mode == 0x00){
                    RCLCPP_INFO(nh_->get_logger(), "change mode: normal mode");
                }else if (mode == 0x01){
                    RCLCPP_INFO(nh_->get_logger(), "change mode: auto aim mode");
                }else{
                    RCLCPP_INFO(nh_->get_logger(), "change mode:  mode err!");
                }
            }
            else{
                //invalid cmd
            }
        }
    }
}
```

