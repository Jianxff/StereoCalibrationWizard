#include "calibrate.hpp"
using namespace std;
using namespace cv;


double Calibrate::stereoCalibrate(){
    if(_count <= 0){
        logging.error("invalid count number : %d\n",_count);
        return 0;
    }else
        logging.info("running stereo calibration with %d images\n",_count);


    if(_conf.second_camera_index < -1){
        logging.warning("no secondary camera connected.");
        return 0;
    }

    vector<vector<Point3f>> _object_buf;
    _object_buf.resize(_count);
    for(int i = 0; i < _count; i++ )
        for(int j = 0; j < _conf.board_size.height; j++ )
            for(int k = 0; k < _conf.board_size.width; k++ )
                _object_buf[i].push_back(Point3f(float(k * _conf.square_size), float(j * _conf.square_size), 0));

    sdata.rms_ste = cv::stereoCalibrate(_object_buf,
                                cdata[0].chessboard_buf, cdata[1].chessboard_buf,
                                cdata[0].camera_matrix, cdata[0].dist_coeffs,
                                cdata[1].camera_matrix, cdata[1].dist_coeffs,
                                _conf.image_size,
                                sdata.R_mat, sdata.T_mat, sdata.E_mat, sdata.F_mat,
                                _conf.calib_flag | CALIB_FIX_INTRINSIC,
                                TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)
                                );

    logging.info("stereo calibration for camera system returned rms %.6f\n",sdata.rms_ste);
    double epi = _computeEpipolarError();
    epi_record.emplace_back(epi);
    // rms_record_ste.emplace_back(sdata.rms_ste);
    return sdata.rms_ste;
}

double Calibrate::_computeEpipolarError(){
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    int npt = _conf.board_size.width * _conf.board_size.height;
    for(int i = 0; i < _count; i++ ){
        for(int j = 0; j < 2; j++){
            Mat imgpt(cdata[j].chessboard_buf[i]);
            cv::undistortPoints(imgpt, imgpt, 
                                cdata[j].camera_matrix, cdata[j].dist_coeffs,
                                Mat(), cdata[j].camera_matrix);
            cv::computeCorrespondEpilines(imgpt, j + 1, sdata.F_mat, lines[j]);
        }
        for(int k = 0; k < npt; k++){
            // lines: ax + by + c, and a^2 + b^2 = 1
            double errik1 = fabs(cdata[0].chessboard_buf[i][k].x * lines[1][k][0] +
                                cdata[0].chessboard_buf[i][k].y * lines[1][k][1] + lines[1][k][2]);
            double errik2 = fabs(cdata[1].chessboard_buf[i][k].x * lines[0][k][0] +
                                cdata[1].chessboard_buf[i][k].y * lines[0][k][1] + lines[0][k][2]);
            double errik = (errik1 * errik1 + errik2 + errik2) / 2;
            err += errik;
        }
        npoints += npt;
    }
    sdata.epi_error = sqrt(err/npoints);
    logging.info("epipolar error calculation returned rms %.6f\n",sdata.epi_error);
    return sdata.epi_error;
}


void Calibrate::stereoRectify(Mat& frameL, Mat& frameR,bool to_file, Mat* outframeL, Mat* outframeR){
    if(_conf.second_camera_index < -1){
        logging.error("no secondary camera connected.");
        return;
    }

    Rect validRoi[2];
    cv::stereoRectify(cdata[0].camera_matrix, cdata[0].dist_coeffs,
                    cdata[1].camera_matrix, cdata[1].dist_coeffs,
                    _conf.image_size,
                    sdata.R_mat, sdata.T_mat,sdata.R1_mat, sdata.R2_mat,
                    sdata.P1_mat, sdata.P2_mat, sdata.Q_mat,
                    CALIB_ZERO_DISPARITY, 0, _conf.image_size, &validRoi[0], &validRoi[1]);
    
    cv::Mat mapL1,mapL2,mapR1,mapR2;
    cv::initUndistortRectifyMap(cdata[0].camera_matrix,cdata[0].dist_coeffs,
                                sdata.R1_mat,sdata.P1_mat,_conf.image_size,CV_16SC2,mapL1,mapL2);
    cv::remap(frameL,(outframeL == nullptr ? frameL : *outframeL), mapL1,mapL2,cv::INTER_LINEAR);

    cv::initUndistortRectifyMap(cdata[1].camera_matrix,cdata[1].dist_coeffs,
                                sdata.R2_mat,sdata.P2_mat,_conf.image_size,CV_16SC2,mapR1,mapR2);
    cv::remap(frameR,(outframeR == nullptr ? frameR : *outframeR), mapR1,mapR2,cv::INTER_LINEAR);

    if(to_file){
        cv::imwrite(_conf.storage_path + "/output/_RL.jpg",(outframeL == nullptr ? frameL : *outframeL));
        cv::imwrite(_conf.storage_path + "/output/_RR.jpg",(outframeR == nullptr ? frameR : *outframeR));
    }
}