#ifndef __MEASURE_HPP__
#define __MEASURE_HPP__

#include "../../basic.hpp"
// #include "../Config/config.hpp"
#include "../Calibrate/calibrate.hpp"
#include "../Capture/capture.hpp"
#include "elas/elas_type.hpp"
#include "sgbm/sgbm_type.hpp"
#include "ADcensus/adcensus_type.hpp"

class Measure{
    int _width,_height;
    int _mode;
    std::string _filepath;
    /* SGBM */
    SGBMOption _sgbm_opt;
    cv::Ptr<cv::StereoSGBM> _sgbm;
    /* ELAS */
    ELASOption _elas_opt;
    Elas::parameters _elas_param;
    /* ADCensus */
    ADCOption _adc_opt;
    ADCensusOption _ad_option;
    ADCensusStereo _ad_census;


    cv::Mat _frameL_bak;
    cv::Mat _frame3D, _frameDisp, _frameDispShow, _frameColor;
    std::vector<cv::Point> _select;
    cv::Point _on_point;

    double _distCount(cv::Point,cv::Point);
    void _computeELAS();
    void _computeSGBM();
    void _computeADCensus();
    static void _onMouse(int event, int x, int y, int flags, void* ustc);

public:
    const enum DISP_MODE{
        ELAS,SGBM,ADCensus
    };
    cv::Mat _frameL, _frameR;
    Measure(Config&);
    void readPoints();
    void init(int mode);
    void compute(cv::Mat& Q_mat);
    void drawDist();
    void showEpi();
    void showMeasure();

};

#endif