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
#include "rm_base/serialport_dev.hpp"

using namespace rm_base;
using namespace std;

bool SerialPortDev::isOpen(){
    return mMcuSerialPort.isOpen();
}

int SerialPortDev::sendData(const unsigned char *send_buf,int data_len){
    return mMcuSerialPort.Send(send_buf,data_len);

}

int SerialPortDev::recvData(unsigned char *recv_buf,int data_len){
    return mMcuSerialPort.Recv(recv_buf,data_len);
}

bool SerialPortDev::init(std::string devPath){
    return mMcuSerialPort.init(devPath);
}
