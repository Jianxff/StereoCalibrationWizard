#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__
#include "../../basic.hpp"


class Config{
public:
    std::string storage_path;       // file storage path
    std::string config_path;
    cv::Size    image_size;
    int main_camera_index = -2;     // main camera (usually left cam)
    int second_camera_index = -2;   // secondary camera

    /* Calibrate Parameters */
    cv::Size    board_size;
    double      square_size;
    double      auto_capture_range;

    bool        fix_aspect_ratio;
    bool        zero_tangent_dist;
    bool        fix_principal_point;
    bool        fix_k1;
    bool        fix_k2;
    int         calib_flag = 0;

    Config(const char* file_path = nullptr);
    void import(const char* file_path);
};

#endif