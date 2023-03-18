#ifndef __TIMER_HPP__
#define __TIMER_HPP__
#include <time.h>

class Timer
{
    clock_t _start = 0;
    clock_t _end = 0;

public:
    void start(){
        _start = clock();
    }

    void end(){
        _end = clock();
    }


    double durationSec(){
        return (double)(_end - _start) / CLOCKS_PER_SEC;
    }

    double durationMin(){
        return (double)(_end - _start) / CLOCKS_PER_SEC / 60;
    }

    void clear(){
        _start = 0;
        _end = 0;
    }

};


#endif