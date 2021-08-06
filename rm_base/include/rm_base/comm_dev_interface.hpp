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
#ifndef RM_BASE_COMM_DEV_INTERFACE_HPP
#define RM_BASE_COMM_DEV_INTERFACE_HPP

namespace rm_base{

//communication device interface for data transmission between embedded systems (stm32,c51) and PC
class CommDevInterface{
public :
    virtual bool isOpen()=0;
    //return recv len>0,return <0 if error 
    virtual int recvData(unsigned char *recv_buf,int data_len)=0;
    //return send len>0,return <0 if error 
    virtual int sendData(const unsigned char *send_buf,int data_len)=0;
};

}
#endif //RM_BASE_COMM_DEV_INTERFACE_HPP