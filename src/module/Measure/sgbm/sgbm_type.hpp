#ifndef __SGBM_TYPE_HPP__
#define __SGBM_TYPE_HPP__
#include <opencv2/opencv.hpp>
#include "../../basic.hpp"

class SGBMOption{
public:
    int min_disparity;
    int p1_num,p2_num;
    int num_disparity;
    int SAD_window_size;
    int pre_filter_cap;
    int uniq_ratio;
    int speckle_range;
    int speckle_window_size;
    int disp_max_diff;
    int mode;
    bool fill_blank;

    void read(const char* filepath){
        try{
            cv::FileStorage fs(filepath, cv::FileStorage::READ);
            cv::FileNode node = fs["sgbm"];
            node["mode"] >> mode;
            node["pre_filter_cap"] >> pre_filter_cap;
            node["min_disparity"] >> min_disparity;
            node["num_disparity"] >> num_disparity;
            node["sad_window_size"] >> SAD_window_size;
            node["uniquenness_ratioe"] >> uniq_ratio;
            node["p1_num"] >> p1_num;
            node["p2_num"] >> p2_num;
            node["speckle_range"] >> speckle_range;
            node["speckle_window_size"] >> speckle_window_size;
            node["disp_max_diff"] >> disp_max_diff;
            node["fill_blank"] >> fill_blank;
            fs.release(); 
        }catch(std::exception& e){
            logging.warning("no config files\n");
        }
    }

    void _setTrackBar(std::string name,int init, int max){
        cv::createTrackbar(name,"SGBM ToolBox",nullptr,max);
        cv::setTrackbarPos(name,"SGBM ToolBox",init);
    }

    void createTrackBar(){
        cv::namedWindow("SGBM ToolBox", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("SGBM ToolBox",cv::Size(480,660));
        _setTrackBar("mode",mode,8);
        _setTrackBar("min_disparity",min_disparity,100);
        _setTrackBar("p1_num",p1_num,20);
        _setTrackBar("p2_num",p2_num,100);
        _setTrackBar("num_disparity",num_disparity,255);
        _setTrackBar("SAD_window_size",SAD_window_size,21);
        _setTrackBar("pre_filter_cap",pre_filter_cap,50);
        _setTrackBar("uniq_ratio",uniq_ratio,200);
        _setTrackBar("speckle_range",speckle_range,200);
        _setTrackBar("speckle_window_size",speckle_window_size,200);
        _setTrackBar("disp_max_diff",disp_max_diff,100);
        _setTrackBar("fill_blank",fill_blank,1);
    }

    void readTrackBar(){
        mode = cv::getTrackbarPos("mode","SGBM ToolBox");
        min_disparity = cv::getTrackbarPos("min_disparity","SGBM ToolBox");
        p1_num = cv::getTrackbarPos("p1_num","SGBM ToolBox"),
        p2_num = cv::getTrackbarPos("p2_num","SGBM ToolBox");
        num_disparity = cv::getTrackbarPos("num_disparity","SGBM ToolBox");
        SAD_window_size = cv::getTrackbarPos("SAD_window_size","SGBM ToolBox");
        pre_filter_cap = cv::getTrackbarPos("pre_filter_cap","SGBM ToolBox");
        uniq_ratio = cv::getTrackbarPos("uniq_ratio","SGBM ToolBox");
        speckle_range = cv::getTrackbarPos("speckle_range","SGBM ToolBox");
        speckle_window_size = cv::getTrackbarPos("speckle_window_size","SGBM ToolBox");
        disp_max_diff = cv::getTrackbarPos("disp_max_diff","SGBM ToolBox");
        fill_blank = cv::getTrackbarPos("fill_blank","SGBM ToolBox");
    }

};


#endif