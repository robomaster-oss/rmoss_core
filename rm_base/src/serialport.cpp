////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for Robot
///ALL RIGHTS RESERVED
///@file:serial_port.cpp
///@brief: ubuntu下 通用串口模块源文件
///实现串口通信。参考RM2017@uestc代码
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#include "rm_base/serialport.h"
#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <sstream>

using namespace std;

//暂不理解以下含义
#define RESET -1
#define SET 0

SerialPort::SerialPort(){
     fd=-1;//默认端口未打开，为-1;
     is_open_=false;
     is_init_para_=false;
}
SerialPort::~SerialPort(void)
{  
    if(fd!=-1){
       close(fd);
     }
}

bool SerialPort::init() {
    fd = open(mPort.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
    if(RESET == fd)
    {
        string strErr="Can't Open Serial Port("+mPort+")";
        perror(strErr.c_str());
        err = e_openfile_t;
        return false;
    }
    //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        err = e_fcntl_t;
    }
    //测试是否为终端设备
    if(0 == isatty( STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        err = e_terminal_t;
    }
    else
    {
        //printf("isatty success!\n");
    }
    //设置串口数据帧格式
    if(paramSet(mSpeed, mFlowCtrl, mDatabits, mStopbits, mParity) == RESET)
    {
        cout << "set param err!!!" << endl;
        err =  e_seterr_t;
    }
    else
    {
        //ShowParam();
        err = e_true_t;
    }
    is_open_=true;
    return true;
}

bool SerialPort::init(std::string port, int speed, int flowCtrl, int databits, int stopbits, int parity)
{
    mPort=port;
    mSpeed = speed;
    mFlowCtrl = flowCtrl;
    mDatabits = databits;
    mStopbits = stopbits;
    mParity = parity;
    is_init_para_=true;
    return init();
}

int SerialPort::paramSet(int m_speed, int m_flow_ctrl, int m_databits, int m_stopbits, int m_parity)
{
    //设置串口数据帧格式
    int speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    //tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.

    if(tcgetattr(fd, &options) != 0)
    {
        perror("Setup Serial err!!!");
        err = e_openseril_t;
    }

    //设置串口输入波特率和输出波特率
    for(size_t i = 0; i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if(m_speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(m_flow_ctrl)
    {
    case 0://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch(m_databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size!!!\n");
        err = e_datasize_t;
    }
    //设置校验位
    switch (m_parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        err = e_parity_t;
    }
    // 设置停止位
    switch(m_stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        err = e_stopbits_t;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);//by gzp,传输特殊字符，否则特殊字符0x0d,0x11,0x13会被屏蔽或映射。
    //    options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if(tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        err = e_activate_t;
    }
    err = e_true_t;
    return 0;
}

int SerialPort::Recv(unsigned char *rcv_buf,int data_len)
{
    int ret;
    ret = read( fd, rcv_buf, data_len);
    Flush();
    //断线重连
    if(ret<=0){
        is_open_=false;
    }
    return ret;
}


int SerialPort::Send(const unsigned char *send_buf,int data_len)
{
    int len = 0;
    len = write(fd,send_buf,data_len);
    if(len == data_len)
    {
        //cout << "send successful!!!" << endl;
        return len;
    }else{
        //断线重连
        is_open_=false;
        tcflush(fd,TCOFLUSH);
        return RESET;
    }
}


bool SerialPort::Flush(void)
{
    tcflush(fd, TCIFLUSH);
    return true;
}

void SerialPort::ShowParam(void)
{
    cout << "fd        = " << fd << endl
         << "uart_port = " << mPort << endl
         << "speed     = " << mSpeed << endl
         << "flow_ctrl = " << mFlowCtrl << endl
         << "databits  = " << mDatabits << endl
         << "stopbits  = " << mStopbits << endl
         << "parity    = " << (char)mParity << endl;
}

bool SerialPort::isOpen(){
    if((!is_open_)&&is_init_para_){
        mut.lock();
        reConnect();
        mut.unlock();
    }
    return is_open_;
}

void SerialPort::reConnect(){
    //断线重连
    cerr<<"[serial port] reconning.........."<<endl;
    usleep(200000);//200ms
    if(fd!=-1){
        close(fd);
    }
    init();
}
