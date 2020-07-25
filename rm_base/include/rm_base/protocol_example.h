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

#ifndef RM_BASE_PROCOTOL_EXAMPLE_H
#define RM_BASE_PROCOTOL_EXAMPLE_H

namespace rm_base{
namespace protocol_example{
    typedef enum:unsigned char {
    Gimbal_Angle_Control=0x01,
    Change_Mode=0xa1
  } ProtocolExample;
} 

}

#endif //RM_BASE_PROCOTOL_EXAMPLE_H