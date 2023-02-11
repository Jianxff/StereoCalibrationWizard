#ifndef __ADCENSUS_HPP__
#define __ADCENSUS_HPP__

#include "../../basic.hpp"
#include "source_code/ADCensusStereo.h"

class ADCOption{
public:
    int min_disparity;
    int max_disparity;
    double lrcheck_thres;
    bool do_lrcheck;
    bool do_filling;

    void setVal(ADCensusOption& p){
        p.min_disparity = min_disparity;
        p.max_disparity = max_disparity;
        p.lrcheck_thres = lrcheck_thres;
        p.do_lr_check = do_lrcheck;
        p.do_filling = do_filling;
    }

    void read(const char* filepath){
        try{
            cv::FileStorage fs(filepath, cv::FileStorage::READ);
            cv::FileNode node = fs["adcensus"];
            node["min_disparity"] >> min_disparity;
            node["max_disparity"] >> max_disparity;
            node["lrcheck_thres"] >> lrcheck_thres;
            node["do_lrcheck"] >> do_lrcheck;
            node["do_filling"] >> do_filling;
            fs.release();
        }catch(std::exception& e){
            logging.warning("no config files\n");
        }
    }

    void _setTrackBar(std::string name,int init, int max){
        cv::createTrackbar(name,"ADCensus ToolBox",nullptr,max);
        cv::setTrackbarPos(name,"ADCensus ToolBox",init);
    }

    void createTrackBar(){
        cv::namedWindow("ADCensus ToolBox", cv::WINDOW_AUTOSIZE);
        // cv::resizeWindow("ADCensus ToolBox",cv::Size(480,660));
        _setTrackBar("min_disp",min_disparity,100);
        _setTrackBar("max_disp",max_disparity,256);
        _setTrackBar("thres",(int)lrcheck_thres*10,100);
        _setTrackBar("lrcheck",do_lrcheck,1);
        _setTrackBar("fill",do_filling,1);
    }

    void readTrackBar(){
        try{
            min_disparity = cv::getTrackbarPos("min_disp","ADCensus ToolBox");
            max_disparity = cv::getTrackbarPos("max_disp","ADCensus ToolBox");
            lrcheck_thres = ((float)cv::getTrackbarPos("thres","ADCensus ToolBox"))/10;
            do_lrcheck = (bool)cv::getTrackbarPos("lrcheck","ADCensus ToolBox");
            do_filling = (bool)cv::getTrackbarPos("fill","ADCensus ToolBox");
        }catch(std::exception& e){
            logging.critical(-1,"trackbar closed\n");
        }
    }
};


#endif