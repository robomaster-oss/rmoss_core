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

#ifndef RM_TOOL_TIME_TOOL_H
#define RM_TOOL_TIME_TOOL_H

#include <chrono>

//author: wyx
//email : 1418555317@qq.com

namespace rm_tool {

//时间工具类
class TimeTool{
public:
    public:
        const static int MILLISECONDS;
        const static int MICROSECONDS;

    public:
        /** 获取当前时间
         * @return: std::chrono::steady_clock::time_point，可用auto接收
         */
        static std::chrono::steady_clock::time_point getCurrTime();

        /** 配合上面获取当前时间的函数，将之前获取的时间传入该函数中，并选择计时单位（默认ms），即可计时
         * @param: pre_time, 开始计时的时间
         * @param: time_units, 计时单位
         * @return: int64_t，过去的时间
         */
        static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& pre_time, int time_units = MILLISECONDS);

        /** 重载计时函数
         * @param: former_time, 计时的开始时间
         * @param: later_time, 计时的结束时间
         */
        static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& former_time, 
            const std::chrono::steady_clock::time_point& later_time, int time_units = MILLISECONDS);
    
};
}

#endif //RM_TOOL_TIME_TOOL_H

