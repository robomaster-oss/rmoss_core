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
#include "rm_base/trans_dev_serialport.h"

using namespace rm_base;
using namespace std;

bool TransDevSerialPort::isOpen(){
    return mMcuSerialPort.isOpen();
}

int TransDevSerialPort::dataSend(unsigned char *send_buf,int data_len){
    return mMcuSerialPort.Send(send_buf,data_len);

}

int TransDevSerialPort::dataRecv(unsigned char *recv_buf,int data_len){
    return mMcuSerialPort.Recv(recv_buf,data_len);
}

int TransDevSerialPort::init(std::string devPath){
    if(mMcuSerialPort.init(devPath)){
        return 0;
    }else{
        return -1;
    }
}
