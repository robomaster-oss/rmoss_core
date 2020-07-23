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
 *  file  : trans_dev_interface.h
 *  brief : 消息传输设备接口
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-1-02
 ***************************************************************************/
#ifndef ROBOT_BASE_TRANS_DEV_INTERFACE_H
#define ROBOT_BASE_TRANS_DEV_INTERFACE_H

namespace robot_base{

class TransDevInterface{
public :
    virtual bool isOpen()=0;
    //return recv len>0,return <0 if error 
    virtual int dataRecv(unsigned char *recv_buf,int data_len)=0;
    //return send len>0,return <0 if error 
    virtual int dataSend(unsigned char *send_buf,int data_len)=0;
};

}
#endif //ROBOT_BASE_TRANS_DEV_INTERFACE_H