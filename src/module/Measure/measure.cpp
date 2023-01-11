#include "measure.hpp"
using namespace std;
using namespace cv;

Measure::Measure(Config& c){
    _width = c.image_size.width;
    _height = c.image_size.height;
    _filepath = c.config_path;
}

double Measure::_distCount(Point p1,Point p2){
    Point3f v1 = _frame3D.at<Point3f>(p1.y,p1.x);
    Point3f v2 = _frame3D.at<Point3f>(p2.y,p2.x);
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2)) / 10;
}


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
            sprintf(dist_s,"%.1f",dist);
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
    }
    namedWindow("Measurement",cv::WINDOW_AUTOSIZE);
    setMouseCallback("Measurement",_onMouse,this);
}


void Measure::compute(cv::Mat Q_mat){
    if(_mode == ELAS)
        _computeELAS();
    else if(_mode == SGBM)
        _computeSGBM();

    _frame3D = Mat(_frameDisp.size(),CV_32FC3);
    reprojectImageTo3D(_frameDisp,_frame3D,Q_mat,false);
}


void Measure::showMeasure(){
    Mat img = Mat(_height, 2 * _width,CV_8UC3);
    Rect rect_left(0,0,_width,_height);
    Rect rect_right(_width,0,_width,_height);
    _frameL_bak.copyTo(img(rect_right));

    if(_frameDepth.cols > 0){
        Mat temp;
        cvtColor(_frameDepth,temp,cv::COLOR_GRAY2RGB);
        temp.copyTo(img(rect_left));
    }

    // show
    cv::imshow("Measurement",img);
}

void Measure::showEpi(){
    Mat img = cv::Mat(_height, 2 * _width,CV_8UC3);
    Rect rect_left(0,0,_width,_height);
    Rect rect_right(_width,0,_width,_height);
    _frameL.copyTo(img(rect_right));
    _frameR.copyTo(img(rect_left));
    for(int i = 0; i < _height; i += 48)
        cv::line(img,cv::Point(0,i),cv::Point(2 * _width,i),Scalar(0,255,0),1,8);
    // show
    cv::imshow("Measurement",img);
}