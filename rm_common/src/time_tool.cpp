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

std::chrono::steady_clock::time_point getCurrTime(){
    return std::chrono::steady_clock::now();
}

int64_t countTimeDuration(const std::chrono::steady_clock::time_point& begin, 
    const std::chrono::steady_clock::time_point& end, TimeUnit unit){
    if(unit == TimeUnit::MICROSECONDS){
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        return duration.count();
    } else if(unit == TimeUnit::MILLISECONDS){
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        return duration.count();
    }
    return 0;
}
