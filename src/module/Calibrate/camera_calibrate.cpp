#include "calibrate.hpp"
using namespace std;
using namespace cv;

/**
 * @brief construction for Calibrate unit
 * 
 * @param c config data
 */
Calibrate::Calibrate(Config& c){
    this->_conf = c;
    if(_conf.main_camera_index >= -1)
        cdata.emplace_back(CameraData(_conf.main_camera_index, true));

    if(_conf.second_camera_index >= -1)
        cdata.emplace_back(CameraData(_conf.second_camera_index, false));
}

void Calibrate::setCount(int count){
    this->_count = count;
}


/**
 * @brief read image data with chessboard
 * 
 * @param count number of images
 * @return true valid chessboard data 
 * @return false invalid chessboard data
 */
bool Calibrate::readImage(int count){
    if(count <= 0){
        logging.error("invalid count number : %d\n",count);
        return false;
    }

    setCount(count);
    bool res1 = _readImage(cdata[0]);
    bool res2 = true;
    if(_conf.second_camera_index >= -1)
        res2 = _readImage(cdata[1]);
    return res1 & res2;
}


/**
 * @brief read image data with chessboard for single camera
 * 
 * @param data camera data
 * @param count number of images
 * @return true chessboard detected
 * @return false no chessboard detected
 */
bool Calibrate::_readImage(CameraData& data){
    data.chessboard_buf.clear();
    // read image data
    string tag = data.primary ? "primary" : "secondary";
    string path = _conf.storage_path + "/images/" + tag;
    for(int i = 1; i <= _count; i++){
        Mat img = imread(path + "(" + to_string(i) + ").jpg");
        vector<Point2f> points;
        if(findChessboardCorners(img, _conf.board_size, points,
                                    cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_ADAPTIVE_THRESH |
                                    cv::CALIB_CB_FILTER_QUADS)){
            Mat img_gray;
            cvtColor(img,img_gray, COLOR_RGB2GRAY);
            cornerSubPix(img_gray, points, Size(5,5),Size(-1,-1),
                        TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            data.chessboard_buf.emplace_back(points);
        
        }else{
            logging.error("chessboard not found at image %s(%d).jpg\n",tag,i);
            return false;
        }
    }
    return true;
}


/**
 * @brief camera caliration uint with multi-thread implement
 * 
 * @param count number of images 
 * @return rms-reprojection error
 */
double Calibrate::cameraCalibrate(){
    if(_count <= 0){
        logging.error("invalid count number : %d\n",_count);
        return false;
    }else
        logging.info("running camera calibration with %d images\n",_count);

    double rms1, rms2 = 0;
    rms1 = _cameraCalibrate(cdata[0]);
    if(_conf.second_camera_index >= -1)
        rms2 = _cameraCalibrate(cdata[1]);

    if(rms1 == -1 || rms2 == -1){
        logging.warning("invalid calibration result\n");
        return -1;
    }

    /* calculate std */
    vector<vector<double>> perview;
    perview.emplace_back(_perviewErrors(cdata[0]));
    if (_conf.second_camera_index >= -1)
        perview.emplace_back(_perviewErrors(cdata[1]));
    double std = calibrateStd(perview);


    /* rms */
    if(rms2 == 0) rms2 = rms1;  // for single camera
    double rms = sqrt((rms1 * rms1 + rms2 * rms2) / 2);
    logging.info("calibration for camera system returned rms %.6f with std %.6f\n",rms,std);
    rms_record.emplace_back(rms);
    std_record.emplace_back(std);
    return rms;
}



/**
 * @brief single camera calibration
 * 
 * @param data camera data
 * @param count number of images
 * @return double rms-reprojection error
 */
double Calibrate::_cameraCalibrate(CameraData& data){
    string tag = data.primary ? "primary" : "secondary";

    data.R_mat.clear();
    data.T_mat.clear();
    data._object_buf.clear();
    // calibrate camera
    data._object_buf.resize(1);
    for(int i = 0; i < _conf.board_size.height; i++)
        for(int j = 0; j < _conf.board_size.width; j++)
            data._object_buf[0].push_back(Point3f(float(j * _conf.square_size), float(i * _conf.square_size), 0));
    data._object_buf.resize(_count,data._object_buf[0]);
    
    data.rms = calibrateCamera(data._object_buf, data.chessboard_buf, _conf.image_size,
                            data.camera_matrix, data.dist_coeffs,
                            data.R_vec, data.T_vec,
                            _conf.calib_flag | CALIB_FIX_K5
                            );
    bool valid = checkRange(data.camera_matrix) && checkRange(data.dist_coeffs);
    if(!valid){
        logging.error("invalid calib data in calibration for %s camera\n",tag);
        return -1;
    }
    
    data.vec2Matrix();
    logging.info("calibration for %s camera returned rms %.6f\n",tag,data.rms);

    data._F_rec.emplace_back(data.camera_matrix.at<double>(0,0));
    data._K1_rec.emplace_back(data.dist_coeffs.at<double>(0,0));
    data._K2_rec.emplace_back(data.dist_coeffs.at<double>(0,1));
    return data.rms;
}

/**
 * @brief calculate perview reprojection errors
 * 
 * @param data 
 * @return vector<double> 
 */
vector<double> Calibrate::_perviewErrors(CameraData& data){
    vector<double> output;
    for(int i = 0; i < data._object_buf.size(); i++){
        vector<Point2f> projection;
        cv::projectPoints(Mat(data._object_buf[i]),data.R_vec[i],data.T_vec[i],
                            data.camera_matrix,data.dist_coeffs,
                            projection);
        double err = cv::norm(data.chessboard_buf[i],projection,CV_L2);
        int n = (int)data._object_buf[i].size();
        output.emplace_back(std::sqrt(err * err / n));
    }
    return output;
}

/**
 * @brief calculate std
 * 
 * @param input 
 * @return double 
 */
double Calibrate::calibrateStd(vector<vector<double>>& input){
    if(input.size() == 0)   return 0;
    double avg = 0, total = 0;
    for(int i = 0; i < input.size(); i++){
        for(int j = 0; j < input[i].size(); j++)
            avg += input[i][j];
        total += input[i].size();
    }
    avg /= total;
    double sum = 0;
    for(int i = 0; i < input.size(); i++)
        for(int j = 0; j < input[i].size(); j++)
            sum += std::pow(input[i][j] - avg, 2);
    return std::sqrt(sum / total);
}

