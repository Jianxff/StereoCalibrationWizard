#ifndef __ELAS_TYPE_HPP__
#define __ELAS_TYPE_HPP__

#include "source_code/elas.h"
#include "../../basic.hpp"

class ELASOption{
public:
    int disp_min = 0;
    int disp_max = 0;
    float support_threshold = 0;
    int support_texture = 0;
    int candidate_stepsize = 0;
    int incon_window_size = 0;
    int incon_threshold = 0;
    int incon_min_support = 0;
    int grid_size = 0;
    float beta = 0;
    float gamma = 0;
    float sigma = 0;
    float sradius = 0;
    int match_texture = 0;
    int lr_threshold = 0;
    float speckle_sim_threshold = 0;
    int speckle_size = 0;
    int ipol_gap_width = 0;
    bool add_corners;
    bool filter_median;
    bool filter_adaptive_mean;
    bool postprocess_only_left;
    bool subsampling;


    void setVal(Elas::parameters& p){
        p.disp_min = disp_min;
        p.disp_max = disp_max;
        p.support_threshold = support_threshold;
        p.support_texture = support_texture;
        p.candidate_stepsize = candidate_stepsize;
        p.incon_window_size = incon_window_size;
        p.incon_threshold = incon_threshold;
        p.incon_min_support = incon_min_support;
        p.grid_size = grid_size;
        p.beta = beta;
        p.gamma = gamma;
        p.sigma = sigma;
        p.sradius = sradius;
        p.match_texture = match_texture;
        p.lr_threshold = lr_threshold;
        p.speckle_sim_threshold = speckle_sim_threshold;
        p.speckle_size = speckle_size;
        p.ipol_gap_width = ipol_gap_width;
        p.add_corners = add_corners;
        p.filter_median = filter_median;
        p.filter_adaptive_mean = filter_adaptive_mean;
        p.postprocess_only_left = postprocess_only_left;
        p.subsampling = subsampling;
    }

    void read(const char* filepath){
        try{
            cv::FileStorage fs(filepath, cv::FileStorage::READ);
            cv::FileNode node = fs["elas"];
            node["disp_min"] >> disp_min;
            node["disp_max"] >> disp_max;
            node["support_threshold"] >> support_threshold;
            node["support_texture"] >> support_texture;
            node["candidate_stepsize"] >> candidate_stepsize;
            node["incon_window_size"] >> incon_window_size;
            node["incon_min_support"] >> incon_min_support;
            node["add_corners"] >> add_corners;
            node["grid_size"] >> grid_size;
            node["beta"] >> beta;
            node["gamma"] >> gamma;
            node["sigma"] >> sigma;
            node["sradius"] >> sradius;
            node["match_texture"] >> match_texture;
            node["lr_threshold"] >> lr_threshold;
            node["speckle_sim_threshold"] >> speckle_sim_threshold;
            node["speckle_size"] >> speckle_size;
            node["ipol_gap_width"] >> ipol_gap_width;
            node["filter_median"] >> filter_median;
            node["filter_adaptive_mean"] >> filter_adaptive_mean;
            node["postprocess_only_left"] >> postprocess_only_left;
            node["subsampling"] >> subsampling;

            fs.release();
        }catch(std::exception& e){
            logging.warning("no config files\n");
        }
    }

    void _setTrackBar(std::string name,int init,int max){
        cv::createTrackbar(name,"ELAS ToolBox",nullptr,max);
        cv::setTrackbarPos(name,"ELAS ToolBox",init);
    }

    void createTrackBar(){
        cv::namedWindow("ELAS ToolBox", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("ELAS ToolBox",cv::Size(480,660));
        _setTrackBar("support_threshold",(int)(support_threshold*100),100);
        _setTrackBar("support_texture",support_texture,50);
        _setTrackBar("candidate_stepsize",candidate_stepsize, 50);
        _setTrackBar("incon_window_size",incon_window_size, 50);
        _setTrackBar("incon_min_support",incon_min_support, 50);
        _setTrackBar("grid_size",grid_size, 50);
        _setTrackBar("beta",(int)(beta * 100), 50);
        _setTrackBar("gamma",(int)(gamma * 10), 200);
        _setTrackBar("sigma",(int)(sigma * 10), 200);
        _setTrackBar("sradius",(int)(sradius * 10), 200);
        _setTrackBar("match_texture",match_texture, 20);
        _setTrackBar("lr_threshold",lr_threshold, 20);
        _setTrackBar("sim_threshold",(int)(speckle_sim_threshold * 10), 200);
        _setTrackBar("speckle_size",speckle_size / 10, 50);
        _setTrackBar("ipol_gap_width",ipol_gap_width / 100, 50);
    }

    void readTrackBar(){
        try{
            support_threshold = (float)cv::getTrackbarPos("support_threshold","ELAS ToolBox") / 100.0;
            support_texture = cv::getTrackbarPos("support_texture","ELAS ToolBox");
            candidate_stepsize = cv::getTrackbarPos("candidate_stepsize","ELAS ToolBox");
            incon_window_size = cv::getTrackbarPos("incon_window_size","ELAS ToolBox");
            incon_min_support = cv::getTrackbarPos("incon_min_support","ELAS ToolBox");
            grid_size = cv::getTrackbarPos("grid_size","ELAS ToolBox");
            beta = (float)cv::getTrackbarPos("beta","ELAS ToolBox") / 100.0;
            gamma = (float)cv::getTrackbarPos("gamma","ELAS ToolBox") / 10.0;
            sigma = (float)cv::getTrackbarPos("sigma","ELAS ToolBox") / 10.0;
            sradius = (float)cv::getTrackbarPos("sigma","ELAS ToolBox") / 10.0;
            match_texture = cv::getTrackbarPos("match_texture","ELAS ToolBox");
            lr_threshold = cv::getTrackbarPos("lr_threshold","ELAS ToolBox");
            speckle_sim_threshold = (float)cv::getTrackbarPos("sim_threshold","ELAS ToolBox") / 10.0;
            speckle_size = cv::getTrackbarPos("speckle_size","ELAS ToolBox") * 10;
            ipol_gap_width = cv::getTrackbarPos("ipol_gap_width","ELAS ToolBox") * 100;
        }catch(std::exception& e){
            logging.critical(-1,"trackbar closed\n");
        }
    }
};

#endif