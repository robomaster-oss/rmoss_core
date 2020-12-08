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
#include "rm_common/log.hpp"

namespace rm_common {
namespace rclcpp_log {

    std::string logger_name = "default";

    Logger logger_info(RCUTILS_LOG_SEVERITY_INFO);
    Logger logger_error(RCUTILS_LOG_SEVERITY_WARN);
    Logger logger_warn(RCUTILS_LOG_SEVERITY_ERROR);
    Logger logger_fatal(RCUTILS_LOG_SEVERITY_FATAL);

    void setLogName(std::string name)
    {
        logger_name = name;
    }

    Logger& loginfo()
    {
        return logger_info;
    }
    Logger& logwarn()
    {
        return logger_warn;
    }
    Logger& logerror()
    {
        return logger_error;
    }
    Logger& logfatal()
    {
        return logger_fatal;
    }
    // for std::endl ,output buffer to logger and clean buffer.
    Logger& Logger::operator<<(std::ostream& (*pf)(std::ostream&))
    {
        RCUTILS_LOG_COND_NAMED(severity_, RCUTILS_LOG_CONDITION_EMPTY, RCUTILS_LOG_CONDITION_EMPTY,
            logger_name.c_str(), stream_buffer_.str().c_str());
        stream_buffer_<<pf; //just to remove warnning(unused parameter 'pf')
        stream_buffer_.str("");
        return *this;
    }
}
}
