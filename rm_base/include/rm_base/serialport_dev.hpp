// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_BASE_SERIALPORT_DEV_HPP
#define RM_BASE_SERIALPORT_DEV_HPP

#include "rm_base/comm_dev_interface.hpp"
#include "rm_base/serialport.h"

namespace rm_base{

//串口数据传输设备，符合通用传输接口。（对象适配器设计模式）
class SerialPortDev:public CommDevInterface{
    public:
        SerialPortDev(){};
        ~SerialPortDev(){};
    public:
        bool init(std::string dev_path="/dev/ttyUSB0");
        virtual bool isOpen();
        virtual int recvData(unsigned char *recv_buf,int data_len);
        virtual int sendData(const unsigned char *send_buf,int data_len);
    private:
        SerialPort mMcuSerialPort;
};


}

#endif //RM_BASE_SERIALPORT_DEV_HPP
