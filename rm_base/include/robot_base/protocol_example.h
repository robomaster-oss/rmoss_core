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
 *  file  : protocol_example.h
 *  brief : 哨兵协议宏定义
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-04-02
 ***************************************************************************/
#ifndef ROBOT_BASE_PROCOTOL_EXAMPLE_H
#define ROBOT_BASE_PROCOTOL_EXAMPLE_H

namespace robot_base{
namespace protocol_example{
    typedef enum:unsigned char {
    Gimbal_Angle_Control=0x01,
    Change_Mode=0xa1
  } ProtocolExample;
} 

}

#endif //ROBOT_BASE_PROCOTOL_EXAMPLE_H