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
#include "rm_common/time_tool.hpp"

using namespace std;
using namespace rm_common;

const int TimeTool::MICROSECONDS = 0;
const int TimeTool::MILLISECONDS = 1;

std::chrono::steady_clock::time_point TimeTool::getCurrTime(){
    return std::chrono::steady_clock::now();
}

int64_t TimeTool::countTimeDuration(const std::chrono::steady_clock::time_point& pre_time, int time_units){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if(time_units == MICROSECONDS){
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time);
        return duration.count();
    } else if(time_units == MILLISECONDS){
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - pre_time);
        return duration.count();
    }
    return 0;
}

int64_t TimeTool::countTimeDuration(const std::chrono::steady_clock::time_point& former_time, 
    const std::chrono::steady_clock::time_point& later_time, int time_units){
    if(time_units == MICROSECONDS){
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(later_time - former_time);
        return duration.count();
    } else if(time_units == MILLISECONDS){
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(later_time - former_time);
        return duration.count();
    }
    return 0;
}
