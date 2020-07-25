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
#ifndef RM_BASE_TRANS_DEV_INTERFACE_H
#define RM_BASE_TRANS_DEV_INTERFACE_H

namespace rm_base{

//消息传输设备接口
class TransDevInterface{
public :
    virtual bool isOpen()=0;
    //return recv len>0,return <0 if error 
    virtual int dataRecv(unsigned char *recv_buf,int data_len)=0;
    //return send len>0,return <0 if error 
    virtual int dataSend(unsigned char *send_buf,int data_len)=0;
};

}
#endif //RM_BASE_TRANS_DEV_INTERFACE_H