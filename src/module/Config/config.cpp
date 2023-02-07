#include "config.hpp"

using namespace std;
using namespace cv;

Config::Config(const char* filepath){
    if(filepath == nullptr)
        return;
    this->import(filepath);
    this->config_path = string(filepath);
}

void Config::import(const char* filepath){
    FileStorage fs(filepath,FileStorage::READ);
    if(!fs.isOpened())
        logging.critical(-1,"configure file not found.\n");

    fs["storage_path"] >> storage_path;
    fs["chessboard_width"] >> board_size.width;
    fs["chessboard_height"] >> board_size.height;
    fs["chessboard_square_size"] >> square_size;
    fs["image_width"] >> image_size.width;
    fs["image_height"] >> image_size.height;
    fs["manual_settings"] >> manual;
    fs["compress_mjpg"] >> mjpg;

    fs["main_camera_index"] >> main_camera_index;
    fs["second_camera_index"] >> second_camera_index;

    fs["auto_capture_range"] >> auto_capture_range;
    fs["free_cap_num"] >>  free_cap_num;

    fs["flag_fix_aspect_ratio"] >> fix_aspect_ratio;
    fs["flag_zero_tangent_dist"] >> zero_tangent_dist;
    fs["flag_fix_principal_point"] >> fix_principal_point;
    fs["flag_fix_k1"] >> fix_k1;
    fs["flag_fix_k2"] >> fix_k2;
    
    if(fix_principal_point) calib_flag |= CALIB_FIX_PRINCIPAL_POINT;
    if(zero_tangent_dist)   calib_flag |= CALIB_ZERO_TANGENT_DIST;
    if(fix_aspect_ratio)    calib_flag |= CALIB_FIX_ASPECT_RATIO;
    if(fix_k1)              calib_flag |= CALIB_FIX_K1;
    if(fix_k2)              calib_flag |= CALIB_FIX_K2;

    if(main_camera_index == second_camera_index)
        dual_sync = true;

    fs.release();
}