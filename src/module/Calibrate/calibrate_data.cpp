#include "calibrate.hpp"
using namespace std;
using namespace cv;

CameraData::CameraData(int index, bool primary){
    this->primary = primary;
    this->camera_index = index;
}

/**
 * @brief convert calibrate data from vector to matrix
 * 
 */
void CameraData::vec2Matrix(){
    Mat bigmat((int)R_vec.size(), 6, R_vec[0].type());
    for(int i = 0; i < R_vec.size(); i++){
        Mat outR;
        Mat r = bigmat(Range(i, i + 1), Range(0, 3)),
            t = bigmat(Range(i, i + 1), Range(3, 6));

        r = R_vec[i].t();
        t = T_vec[i].t();
        cv::Rodrigues(r, outR);
        R_mat.emplace_back(Mat(outR));
        T_mat.emplace_back(Mat(t));
    }
}

void CameraData::importData(FileStorage& fs, int limit){
    string tag = primary ? "primary" : "secondary";
    
    FileNode fn = fs[tag];
    if(fn.isNone() || fn.empty())
        logging.debug("no exsit data for %s camera\n",tag);
    else{
        fn["camera_matrix"] >> camera_matrix;
        fn["dist_coeff"] >> dist_coeffs;
    }

    FileNode fn2 = fs["record"];
    if(!fn2.isNone() && !fn2.empty()){
        FileNode fn3 = fn2[tag];
        if(fn3.isNone() || fn3.empty())
            logging.debug("no records for %s camera\n",tag);
        else{
            int c1 = 0, c2 = 0, c3 = 0;
            for(auto& it:fn3["F"])
                (++c1 <= limit) && _F_rec.emplace_back(it.real());
            for(auto& it:fn3["K1"])
                (++c2 <= limit) && _K1_rec.emplace_back(it.real());
            for(auto& it:fn3["K2"])
                (++c3 <= limit) && _K2_rec.emplace_back(it.real());
        }
    }
}

void Calibrate::clearData(){
    FileStorage fs(_conf.storage_path + "/calib_data.xml", FileStorage::WRITE);
    fs.release();

    ofstream fout(_conf.storage_path + "/primary.txt",ios::out);
    fout.close();

    if(_conf.second_camera_index >= -1){
        ofstream fout2(_conf.storage_path + "/secondary.txt",ios::out);
        fout2.close();
        ofstream fout3(_conf.storage_path + "/stereo.txt",ios::out);
        fout3.close();
    }
}

void Calibrate::importData(int limit){
    if(limit < 0)
        return;

    FileStorage fs(_conf.storage_path + "/calib_data.xml", FileStorage::READ);
    if(!fs.isOpened()){
        logging.warning("no record file\n");
        return;
    }
    for(auto& cd : cdata)
        cd.importData(fs,limit);
    
    int c1 = 0, c2 = 0;
    FileNode fn0 = fs["record"];
    if(!fn0.isNone() && !fn0.empty()){
        FileNode fn1 = fn0["rms"];
        if(!fn1.isNone() && !fn1.empty())
            for(auto& it:fn1)
                (++c1 <= limit) && rms_record.emplace_back(it.real());
    }
    

    if(_conf.second_camera_index >= -1){
        FileNode fn3 = fs["stereo"];
        if(!fn3.isNone() && !fn3.empty()){
            fn3["R"] >> sdata.R_mat;
            fn3["T"] >> sdata.T_mat;
        }

        if(!fn0.isNone() && !fn0.empty()){
            FileNode fn2 = fn0["epi"];
            if(!fn2.isNone() && !fn2.empty()){
                for(auto& it:fn2)
                    (++c2 <= limit) && epi_record.emplace_back(it.real());
            }else
                logging.debug("no record for stereo camera system\n");
        }
    }
}

void Calibrate::storeData(){
    // cameras  :   matrix dist_coeff R_mat T_mat
    // stereo   :   R_mat T_mat F_mat
    FileStorage fs(_conf.storage_path + "/calib_data.xml", FileStorage::WRITE);
    for(auto& cam : cdata){
        string tag = cam.primary ? "primary" : "secondary";
        fs << tag << "{";
            fs << "index" << cam.camera_index;
            fs << "camera_matrix" << cam.camera_matrix;
            fs << "dist_coeffs" << cam.dist_coeffs;
            fs << "rms" << cam.rms;
        fs << "}";
        ofstream fout(_conf.storage_path + "/output/" + tag + ".txt",ios::out);
        fout << cam.camera_matrix.reshape(0,1) << endl << endl;
        fout << cam.dist_coeffs << endl << endl;
        for(int i = 0; i < _count; i++)
            fout << cam.R_mat[i].reshape(0,1) << endl;
        fout << endl;
        for(int i = 0; i < _count; i++)
            fout << cam.T_mat[i] << endl;
        fout.close();
    }

    ofstream fout(_conf.storage_path + "/output/stereo.txt",ios::out);
    if(_conf.second_camera_index >= -1){
            fs << "stereo" << "{";
            fs << "epi" << sdata.epi_error;
            fs << "R" << sdata.R_mat;
            fs << "T" << sdata.T_mat;
            fs << "E" << sdata.E_mat;
            fs << "F" << sdata.F_mat;
        fs << "}";
        
        fout << sdata.R_mat.reshape(0,1) << endl;
        fout << sdata.T_mat.reshape(0,1) << endl;
        fout << sdata.F_mat.reshape(0,1) << endl;
    }
    fout.close();

    fs << "record" << "{";
        fs << "rms" << rms_record;
        fs << "epi" << epi_record;
        for(auto& cam : cdata){
            string tag = cam.primary ? "primary" : "secondary";
            fs << tag << "{";
                fs << "F" << cam._F_rec;
                fs << "K1" << cam._K1_rec;
                fs << "K2" << cam._K2_rec;
            fs << "}";
        }
        
    fs << "}";
    fs.release();
}
