/****************************************************************************
 *  Copyright (C) 2019 RoboMasterOS.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : trans_dev_serialport.cpp
 *  brief : 消息传输设备具体实现
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-1-02
 ***************************************************************************/
#include "robot_base/serialport_trans_dev.h"

using namespace robot_base;
using namespace std;


SerialPortTransDev::SerialPortTransDev(){


}

SerialPortTransDev::~SerialPortTransDev(){


}

bool SerialPortTransDev::isOpen(){
    return mMcuSerialPort.isOpen();
}


int SerialPortTransDev::dataSend(unsigned char *send_buf,int data_len){
    return mMcuSerialPort.Send(send_buf,data_len);

}


int SerialPortTransDev::dataRecv(unsigned char *recv_buf,int data_len){
    return mMcuSerialPort.Recv(recv_buf,data_len);
}


int SerialPortTransDev::init(std::string devPath){
    if(mMcuSerialPort.init(devPath)){
        return 0;
    }else{
        return -1;
    }
}
