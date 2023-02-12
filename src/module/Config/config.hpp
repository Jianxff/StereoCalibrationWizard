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

    bool manual = false;
    bool mjpg = false;
    int magnify_origin = 10;
    
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
    int         free_cap_num = 0;
    bool        dual_sync = false;

    Config(const char* file_path = nullptr);
    void import(const char* file_path);
};

#endif