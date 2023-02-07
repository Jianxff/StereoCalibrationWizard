/* ============================================================================

    .__                       .__                
    |  |   ____   ____   ____ |__| ____    ____  
    |  |  /  _ \ / ___\ / ___\|  |/    \  / ___\ 
    |  |_(  <_> ) /_/  > /_/  >  |   |  \/ /_/  >
    |____/\____/\___  /\___  /|__|___|  /\___  / 
               /_____//_____/         \//_____/  
    
 
 Uniform Logging Method for Modern C++ 
 Version 1.0
 https://github.com/Jianxff/ToolKits

 Copyright 2023 Jianxff

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

============================================================================ */

#ifndef __LOGGING_HPP__
#define __LOGGING_HPP__

#include <iostream>
#include <stdarg.h>

class Logging{
public:
    static int _level;
    std::string _tag;
    enum Level{
        L_CRITICAL,
        L_ERROR,
        L_WARNING,
        L_INFO,
        L_DEBUG,
    };
    Logging(std::string tag){
        _level = L_INFO;
        _tag = tag;
    }

    void config(Level level){
        _level = level;
    }
    // log time
    void _time(){
        // get current time
        time_t timer;
        time(&timer);
        struct tm tdata;
        localtime_s(&tdata, &timer);
        printf("%04d-%02d-%02d %02d:%02d:%02d", 
                tdata.tm_year + 1900, tdata.tm_mon + 1, tdata.tm_mday,
                tdata.tm_hour, tdata.tm_min, tdata.tm_sec);
    }
    // log debug
    void debug(const char* format,...){
        if(_level < L_DEBUG) return;
        _time();
        printf(" - DEBUG - %s : ",_tag.c_str());
        // get var args
        va_list args;
        va_start(args,format);
        vprintf(format,args);
        va_end(args);
    }

    // log info
    void info(const char* format,...){
        if(_level < L_INFO) return;
        _time();
        printf(" - INFO - %s : ",_tag.c_str());
        // get var args
        va_list args;
        va_start(args,format);
        vprintf(format,args);
        va_end(args);
    }

    // log warning
    void warning(const char* format, ...){
        if(_level < L_WARNING) return;
        _time();
        printf(" - WARNING - %s : ",_tag.c_str());
        // get var args
        va_list args;
        va_start(args,format);
        vprintf(format,args);
        va_end(args);
    }

    // log error
    void error(const char* format, ...){
        if(_level < L_ERROR) return;
        _time();
        printf(" - ERROR - %s : ",_tag.c_str());
        // get var args
        va_list args;
        va_start(args,format);
        vprintf(format,args);
        va_end(args);
    }

    // log critical
    void critical(int exit_code,const char* format, ...){
        _time();
        printf(" - CRITICAL - %s : ",_tag.c_str());
        // get var args
        va_list args;
        va_start(args,format);
        vprintf(format,args);
        va_end(args);
        exit(exit_code);
    }
};

static Logging logging("main.cpp");


#endif
