#include "measure.hpp"
#include "sgbm/sgbm_util.hpp"
using namespace std;
using namespace cv;

void Measure::_computeELAS(){
    /* get settings */
    _elas_opt.readTrackBar();
    _elas_opt.setVal(_elas_param);

    /* compute disp */
    Mat disp = Mat::zeros(Size(_width,_height), CV_32FC1);
    Mat disp_r = Mat::zeros(Size(_width,_height), CV_32FC1);
    Mat img_l,img_r,_img_l,_img_r, depth;
    int dim[3] = {_width,_height,_width};

    // _frameL.copyTo(img_l);
    // _frameR.copyTo(img_r);
    cvtColor(_frameL,img_l,COLOR_BGR2GRAY);
    cvtColor(_frameR,img_r,COLOR_BGR2GRAY);
    GaussianBlur(img_l,img_l,Size(3,3),0);
    GaussianBlur(img_r,img_r,Size(3,3),0);

    // elas process
    Elas elas(_elas_param);
    elas.process(img_l.data,img_r.data,disp.ptr<float>(0),disp_r.ptr<float>(0),dim);

    disp.convertTo(_frameDisp,CV_32F);
    disp.convertTo(_frameDispShow,CV_8UC1);
    //disp.copyTo(_frameDisp);
    // depth
    //normalize(disp,_frameDepth,0,255,cv::NORM_MINMAX,CV_8U);
    applyColorMap(_frameDispShow,_frameColor,COLORMAP_JET);       
}

void Measure::_computeSGBM(){
    _sgbm_opt.readTrackBar();
    switch(_sgbm_opt.mode){
        case 3:     _sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);break;
        case 4:     _sgbm->setMode(cv::StereoSGBM::MODE_HH4);break;
        case 8:     _sgbm->setMode(cv::StereoSGBM::MODE_HH);break;
        default:    _sgbm->setMode(cv::StereoSGBM::MODE_SGBM);break;
    }
    _sgbm->setMinDisparity(_sgbm_opt.min_disparity);
    _sgbm->setPreFilterCap(_sgbm_opt.pre_filter_cap);
    _sgbm->setNumDisparities(_sgbm_opt.num_disparity);
    _sgbm->setBlockSize(_sgbm_opt.SAD_window_size);
    int cn = _frameL.channels();
    _sgbm->setP1(_sgbm_opt.p1_num * cn * _sgbm_opt.SAD_window_size * _sgbm_opt.SAD_window_size);
    _sgbm->setP2(_sgbm_opt.p2_num * cn * _sgbm_opt.SAD_window_size * _sgbm_opt.SAD_window_size);
    _sgbm->setUniquenessRatio(_sgbm_opt.uniq_ratio);
    _sgbm->setSpeckleRange(_sgbm_opt.speckle_range);
    _sgbm->setSpeckleWindowSize(_sgbm_opt.speckle_window_size);
    _sgbm->setDisp12MaxDiff(_sgbm_opt.disp_max_diff);

    /* Compute */
    Mat img_l,img_r,_img_l,_img_r, disp, depth;
    // _frameL.copyTo(img_l);
    // _frameR.copyTo(img_r);
    cvtColor(_frameL,img_l,COLOR_BGR2GRAY);
    cvtColor(_frameR,img_r,COLOR_BGR2GRAY);
    GaussianBlur(img_l,img_l,Size(3,3),0);
    GaussianBlur(img_r,img_r,Size(3,3),0);

    _sgbm->compute(img_l,img_r,disp);
    if(_sgbm_opt.fill_blank)
        fillBlank(disp);

    disp.convertTo(_frameDisp,CV_32F,1.0/16);
    // disp.copyTo(_frameDispShow);
    // Mat disp8u;
    _frameDisp.convertTo(_frameDispShow,CV_8UC1);
    // depth
    // cv::normalize(disp,_frameDepth,0,255,cv::NORM_MINMAX,CV_8U);
    cv::applyColorMap(_frameDispShow,_frameColor,COLORMAP_JET);        
}

void Measure::_computeADCensus(){
    _adc_opt.readTrackBar();
    _adc_opt.setVal(_ad_option);
    
    _ad_census.Initialize(_width,_height,_ad_option);

    // 左右影像的彩色数据
    auto bytes_left = new uint8[_width * _height * 3];
    auto bytes_right = new uint8[_width * _height * 3];
    for (int i = 0; i < _height; i++) {
        for (int j = 0; j < _width; j++) {
            bytes_left[i * 3 * _width + 3 * j] = _frameL.at<cv::Vec3b>(i, j)[0];
            bytes_left[i * 3 * _width + 3 * j + 1] = _frameL.at<cv::Vec3b>(i, j)[1];
            bytes_left[i * 3 * _width + 3 * j + 2] = _frameL.at<cv::Vec3b>(i, j)[2];
            bytes_right[i * 3 * _width + 3 * j] = _frameR.at<cv::Vec3b>(i, j)[0];
            bytes_right[i * 3 * _width + 3 * j + 1] = _frameR.at<cv::Vec3b>(i, j)[1];
            bytes_right[i * 3 * _width + 3 * j + 2] = _frameR.at<cv::Vec3b>(i, j)[2];
        }
    }

    auto disparity = new float32[uint32(_width * _height)]();
    int res = _ad_census.Match(bytes_left, bytes_right, disparity);
    if(!res){
        logging.error("ADCensus match failed.");
        return;
    }

    /* Convert OpenCv */
    Mat disp_show = Mat(Size(_width,_height), CV_8UC1);
    Mat disp_count = Mat(Size(_width,_height), CV_8UC1);
    float32 min_disp = float32(_width), max_disp = -float32(_width);
    for (sint32 i = 0; i < _height; i++) {
        for (sint32 j = 0; j < _width; j++) {
            const float32 disp = abs(disparity[i * _width + j]);
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (sint32 i = 0; i < _height; i++) {
        for (sint32 j = 0; j < _width; j++) {
            const float32 disp = abs(disparity[i * _width + j]);
            if (disp == Invalid_Float)
                disp_show.data[i * _width + j] = disp_count.data[i * _width + j] = 0;
            else{ 
                disp_show.data[i * _width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
                disp_count.data[i * _width + j] = disp;
            }
        }
    }
    
    //disp_show.convertTo(_frameDisp,CV_32F);
    disp_show.copyTo(_frameDispShow);
    disp_count.convertTo(_frameDisp,CV_32F);
    // disp_show.convertTo(_frameDisp,CV_32F);
    // depth
    // normalize(disp,_frameDepth,0,255,cv::NORM_MINMAX,CV_8U);
    applyColorMap(disp_show,_frameColor,COLORMAP_JET);
    delete disparity;
    delete bytes_left;
    delete bytes_right;
}