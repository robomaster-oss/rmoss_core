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
#ifndef RM_COMMON_LOG_HPP
#define RM_COMMON_LOG_HPP

#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

//log system: 
// #define USE_RCLCPP_LOG, for Rclcpp Log
// #define USE_GLOG, for Glog
// #define USE_NO_LOG, for no Log

// define Rclcpp Log if not define (default log setting)
#if (!defined(USE_RCLCPP_LOG) && !defined(USE_GLOG) && !defined(USE_NO_LOG) )
#define USE_RCLCPP_LOG
#endif

//choose log system
#ifdef USE_RCLCPP_LOG

#define RM_LOG_INFO rm_common::rclcpp_log::loginfo()
#define RM_LOG_WARN rm_common::rclcpp_log::logwarn()
#define RM_LOG_ERROR rm_common::rclcpp_log::logerror()
#define RM_LOG_FALTA rm_common::rclcpp_log::logfatal()
#endif

//glog is not supported now, TODO...
#ifdef USE_GLOG
#define RM_LOG_INFO LOG(INFO)
#define RM_LOG_WARN LOG(WARN)
#define RM_LOG_ERROR LOG(ERROR)
#define RM_LOG_FALTA LOG(FALTA)
#endif

//no log system
#ifdef USE_NO_LOG
#define RM_LOG_INFO std::cout
#define RM_LOG_WARN std::cerr
#define RM_LOG_ERROR std::cerr
#define RM_LOG_FALTA std::cerr
#endif

namespace rm_common {

//a simple stream wrapper for rclcpp logger 
namespace rclcpp_log {
    void setLogName(std::string name);
    //reference:https://stackoverflow.com/questions/25615253/correct-template-method-to-wrap-a-ostream
    class Logger {
    private:
        //for rclcpp logger
        int severity_ { RCUTILS_LOG_SEVERITY_INFO };
        std::stringstream stream_buffer_;
    public:
        Logger(int severity)
            : severity_(severity) {};
        //store to buffer
        template <class T>
        Logger& operator<<(const T& t)
        {
            stream_buffer_ << t;
            return *this;
        }
        // for endl ,output buffer to logger and clean buffer.
        Logger& operator<<(std::ostream& (*pf)(std::ostream&));
    };
    Logger& loginfo();
    Logger& logwarn();
    Logger& logerror();
    Logger& logfatal();
} //namespace rclcpp_log
} // namespace rm_common

#endif // RM_COMMON_LOG_HPP
