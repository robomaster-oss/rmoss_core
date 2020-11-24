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

#ifndef RM_COMMON_TIME_TOOL_HPP
#define RM_COMMON_TIME_TOOL_HPP

#include <chrono>

//author: wyx
//email : 1418555317@qq.com

namespace rm_common {
enum class TimeUnit {
    MICROSECONDS, //us
    MILLISECONDS, //ms
};
//时间工具
/** 获取当前时间
* @return: std::chrono::steady_clock::time_point，可用auto接收
*/
std::chrono::steady_clock::time_point getCurrTime();

/** 重载计时函数
* @param: begin, 计时的开始时间
* @param: end, 计时的结束时间
*/
int64_t countTimeDuration(const std::chrono::steady_clock::time_point& begin,
    const std::chrono::steady_clock::time_point& end, TimeUnit unit = TimeUnit::MILLISECONDS);

}

#endif //RM_COMMON_TIME_TOOL_HPP
