#ifndef __CALIBRATE_HPP__
#define __CALIBRATE_HPP__

#include "../../basic.hpp"
#include "../Config/config.hpp"

class CameraData{
friend class Calibrate;
    std::vector<std::vector<cv::Point3f>>   _object_buf;
    std::vector<double>          _F_rec,_K1_rec,_K2_rec;
public:
    CameraData(int index, bool primary = true);
    void importData(cv::FileStorage&, int limit = INT_MAX);
    void vec2Matrix();
    int                         camera_index;
    bool                        primary;
    std::vector<std::vector<cv::Point2f>>   chessboard_buf;
    std::vector<cv::Mat>        R_vec, R_mat;
    std::vector<cv::Mat>        T_vec, T_mat;
    cv::Mat                     camera_matrix, dist_coeffs;
    double rms;
};

class StereoData{
friend class Calibrate;
public:
    cv::Mat R_mat, T_mat;
    cv::Mat E_mat, F_mat;
    cv::Mat R1_mat, R2_mat, P1_mat, P2_mat;
    cv::Mat Q_mat;
    double rms;
    double epi_error;
};



class Calibrate{
    Config _conf;
    bool    _readImage(CameraData&);
    double  _cameraCalibrate(CameraData&);
    double  _computeEpipolarError();
    int     _count;

public:
    std::vector<CameraData>     cdata;
    StereoData                  sdata;

    std::vector<double>         rms_record;
    std::vector<double>         epi_record;
    
    Calibrate(Config&);
    void    clearData();
    void    importData(int limit = INT_MAX);
    void    setCount(int count);        
    bool    readImage(int count);       
    double  cameraCalibrate();
    double  stereoCalibrate();
    void    stereoRectify(cv::Mat* frameL = nullptr, cv::Mat* frameR = nullptr,
                        cv::Mat* outframeL = nullptr, cv::Mat* outframeR = nullptr);
    void    storeData();
};



#endif