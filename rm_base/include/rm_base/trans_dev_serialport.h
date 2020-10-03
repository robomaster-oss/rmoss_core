/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#ifndef RM_BASE_TRANS_DEV_SERIALPORT_H
#define RM_BASE_TRANS_DEV_SERIALPORT_H

#include "rm_base/trans_dev_interface.h"
#include "rm_base/serial_port.h"

namespace rm_base{

//串口数据传输设备，符合通用传输接口。（对象适配器设计模式）
class TransDevSerialPort:public TransDevInterface{
    public:
        TransDevSerialPort(){};
        ~TransDevSerialPort(){};
    public:
        int init(std::string dev_path="/dev/ttyUSB0");
        bool isOpen();
        int dataRecv(unsigned char *recv_buf,int data_len);
        int dataSend(unsigned char *send_buf,int data_len);
    private:
        SerialPort mMcuSerialPort;
};


}

#endif //RM_BASE_TRANS_DEV_SERIALPORT_H
