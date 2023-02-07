#ifndef __FUNCTION_HPP__
#define __FUNCTION_HPP__

#include "../module/Config/config.hpp"
#include "../module/Capture/capture.hpp"
#include "../module/Calibrate/calibrate.hpp"
#include "../module/Measure/measure.hpp"


void    listCamera();
int     initCalibrate();
int     nextCalibrate();
int     freeCalibrate(int total = 50);
int     measure(int mode = Measure::ELAS);
int     measureRT(int mode = Measure::ELAS);

#endif