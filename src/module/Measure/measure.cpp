#include "measure.hpp"
using namespace std;
using namespace cv;

Measure::Measure(Config& c){
    _width = c.image_size.width;
    _height = c.image_size.height;
    _filepath = c.config_path;
}

void Measure::readPoints(){
    FileStorage fs(_filepath,FileStorage::READ);
    if(!fs.isOpened())
        logging.critical(-1,"configure file not found.\n");

    FileNode fn = fs["select_points"];
    if(fn.isNone() || fn.empty()){
        logging.debug("no selected points\n");
        return;
    }
    bool xy = false;
    Point p = Point(0,0);
    for(auto& it:fn){
        if(!xy){
            p.x = (int)it.real();
            xy = true;
        }else{
            p.y = (int)it.real();
            xy = false;
            _select.emplace_back(Point(p.x,p.y));
        }
    }
    if(xy){
        _select.clear();
        logging.error("points data incomplete\n");
    }

}

double Measure::_distCount(Point p1,Point p2){
    Point3f v1 = _frame3D.at<Point3f>(p1.y,p1.x);
    Point3f v2 = _frame3D.at<Point3f>(p2.y,p2.x);
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2)) / 10;
}


//cv::setMouseCallback("Measurement",_onMouse,this);
void Measure::_onMouse(int event, int x, int y, int flags, void* ustc){
    Measure* m = (Measure*)ustc;
    // limit
    x -= m->_width;
    if(x < 0 || x > m->_width || y < 0 || y > m->_width){
        m->_on_point = Point(-1,-1);
        return;
    }
    m->_on_point = Point(x, y);
    if(event == EVENT_RBUTTONDBLCLK){       // mid button
        m->_select.clear();
    }
    else if(event == EVENT_RBUTTONDOWN){    // right button
        if(m->_select.size() <= 2)
            m->_select.clear();
        else
            m->_select.pop_back();
    }
    else if(event == cv::EVENT_LBUTTONDOWN){    // left button
        // chekc range
        logging.debug("click on (%d, %d)\n",x,y);
        m->_select.emplace_back(Point(x,y));
    }
}


void Measure::drawDist(){
    _frameL.copyTo(_frameL_bak);
    if(_select.size() < 2)
        return;
    char dist_s[16];
    for (int i = 1; i < _select.size(); i++){
        Point p1 = _select[i - 1], p2 = _select[i];
        line(_frameL_bak, p1, p2, CV_RGB(0,255,255), 2);
        double dist = _distCount(p1, p2);
        if(dist > 0 && dist < 1000){
            Point mid((p1.x + p2.x)/2,(p1.y + p2.y)/2 + 10);
            sprintf(dist_s,"%.2f",dist);
            putText(_frameL_bak,dist_s,mid,FONT_HERSHEY_SIMPLEX, 0.52, CV_RGB(255,128,0),2);
        }
    }
}

void Measure::init(int mode){
    _mode = mode;
    if(mode == ELAS){
        _elas_opt.read(_filepath.c_str());
        _elas_opt.setVal(_elas_param);
        _elas_opt.createTrackBar();
    }else if(mode == SGBM){
        _sgbm_opt.read(_filepath.c_str());
        _sgbm = StereoSGBM::create(_sgbm_opt.min_disparity,_sgbm_opt.num_disparity,_sgbm_opt.SAD_window_size);
        _sgbm_opt.createTrackBar();
    }else if(mode == ADCensus){
        _adc_opt.read(_filepath.c_str());
        _adc_opt.setVal(_ad_option);
        _adc_opt.createTrackBar();
    }
    namedWindow("Measurement",cv::WINDOW_AUTOSIZE);
    //setMouseCallback("Measurement",_onMouse,this);
}


void Measure::compute(cv::Mat& Q_mat){
    if(_mode == ELAS)
        _computeELAS();
    else if(_mode == SGBM)
        _computeSGBM();
    else if(_mode == ADCensus)
        _computeADCensus();

    setMouseCallback("Measurement",_onMouse,this);
    reprojectImageTo3D(_frameDisp,_frame3D,Q_mat);
}


void Measure::showMeasure(){
    Mat img = Mat(_height, 2 * _width,CV_8UC3);
    Rect rect_left(0,0,_width,_height);
    Rect rect_right(_width,0,_width,_height);
    
    char pos_s[20];
    sprintf(pos_s,"(%d, %d)",_on_point.x,_on_point.y);
    cv::putText(_frameL_bak,pos_s,Point(20,20),FONT_HERSHEY_SIMPLEX, 0.52, CV_RGB(100,149,237),1);
    _frameL_bak.copyTo(img(rect_right));

    if(_frameDispShow.cols > 0){
        Mat temp;
        cvtColor(_frameDispShow,temp,COLOR_GRAY2RGB);
        temp.copyTo(img(rect_left));
        //_frameColor.copyTo(img(rect_left));
    }

    // show
    cv::imshow("Measurement",img);
    // if(_frameDisp.cols > 0)
    //     cv::imshow("Disparity",_frameDisp);
}

void Measure::showEpi(){
    Mat img = cv::Mat(_height, 2 * _width,CV_8UC3);
    Rect rect_left(0,0,_width,_height);
    Rect rect_right(_width,0,_width,_height);
    _frameL.copyTo(img(rect_left));
    _frameR.copyTo(img(rect_right));
    for(int i = 0; i < _height; i += 48)
        cv::line(img,cv::Point(0,i),cv::Point(2 * _width,i),Scalar(0,255,0),1,8);
    // show
    cv::imshow("Measurement",img);
}