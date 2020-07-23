////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for Robot
///ALL RIGHTS RESERVED
///@file:serial_port.h
///@brief: ubuntu 下通用串口模块头文件
///实现串口通信。参考RM2017@uestc代码
///@vesion 1.0
///@author: Gezp
///@email: 1350824033@qq.com
///@date: 2017.12.8
///修订历史：
/// 12.19: 参考2017RM及网上开源代码
///2018.3.9:更改数据帧协议，哨兵为16Byte数据帧。
///2018.4.28：清理程序，串口通用模块，不再包含协议部分。
///2018.4.29:增加重连函数，通过遍端口1-10实现连接串口。
///2018.5.17：取消重连函数，保留重连方法，在收发数据时，异常时会自动重连。
/// （udev固定设备路径，串口发送失败，重连即可）
////////////////////////////////////////////////////////////////////////////////

#ifndef __ROBOT_BASE_SERIAL_PORT_H
#define __ROBOT_BASE_SERIAL_PORT_H

#include <iostream>
#include <mutex>

typedef enum error
{
    e_true_t = 0,
    e_openfile_t,
    e_fcntl_t,
    e_terminal_t,
    e_openseril_t,
    e_datasize_t,
    e_parity_t,
    e_stopbits_t,
    e_activate_t,
    e_seterr_t

}Serial_error_t;



class SerialPort
{
public:
    SerialPort(void);
    ~SerialPort(void);

private:
    bool init();
    int paramSet(int m_speed, int m_flow_ctrl, int m_databits, int m_stopbits, int m_parity);
    bool Flush(void);
public:
    bool init(std::string port,int speed=115200,int flowCtrl=0,int databits=8,int stopbits=1,int m_parity='N');
    int Recv(unsigned char *rcv_buf,int data_len);
    int Send(unsigned char *send_buf,int data_len);
    void ShowParam(void);
    bool isOpen();
    void reConnect();

private:
    Serial_error_t err;
    std::string mPort;   //端口设备名
    int mSpeed;
    int mFlowCtrl;
    int fd;
    int mDatabits;
    int mStopbits;
    int mParity;

    bool is_init_para_;
    bool is_open_;

    std::mutex mut;
};

#endif //__ROBOT_BASE_SERIAL_PORT_H
