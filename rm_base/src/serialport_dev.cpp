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
#include "rm_base/serialport_dev.hpp"

using namespace rm_base;
using namespace std;

bool SerialPortDev::isOpen(){
    return mMcuSerialPort.isOpen();
}

int SerialPortDev::dataSend(const unsigned char *send_buf,int data_len){
    return mMcuSerialPort.Send(send_buf,data_len);

}

int SerialPortDev::dataRecv(unsigned char *recv_buf,int data_len){
    return mMcuSerialPort.Recv(recv_buf,data_len);
}

bool SerialPortDev::init(std::string devPath){
    return mMcuSerialPort.init(devPath);
}
