#ifndef __FUNCTION_HPP__
#define __FUNCTION_HPP__

#include "../module/Config/config.hpp"
#include "../module/Capture/capture.hpp"
#include "../module/Calibrate/calibrate.hpp"
#include "../module/Measure/measure.hpp"
#include "../util/timer.hpp"


void    listCamera();
int     initCalibrate();
int     nextCalibrate();
int     freeCalibrate(int total = 50);
int     inputCalibrate();
int     measure(int mode = Measure::ELAS, bool rt = false);
int     measureStatic(int mode);
int     measureRealTime(int mode);

#endif