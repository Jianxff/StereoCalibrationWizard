#include "capture.hpp"
using namespace std;
using namespace cv;

Capture::Capture(){_count_cyc = 0;}

Capture::Capture(Config& c){
    this->_conf = c;
    _count_cyc = 0;

    _camera_list.emplace_back(Camera(_conf.main_camera_index));
    if(_conf.second_camera_index >= -1)
        _camera_list.emplace_back(Camera(_conf.second_camera_index, false));
}

void Capture::openCamera(){
    if(_conf.main_camera_index <= -1 && _conf.second_camera_index <= -1)
        return;
    
    for(auto& cam : _camera_list){
        string tag = cam.primary ? "primary" : "secondary";
        if(!cam.init(_conf))
            logging.critical(-1,"failed connecting to %s camera with index %d\n",tag.c_str(),cam.index);
        else
            logging.info("connect to %s camera with index %d\n",tag.c_str(),cam.index);
        waitKey(300);
    }
    readCount();
    waitKey(1000);
}


void Capture::_showFrame(){
    if(_conf.main_camera_index <= -1)
        return;
    if(_conf.second_camera_index <= -1){
        // single camera
        imshow("Camera System", _camera_list[0].frame);
    }else{
        // dual camera
        int h_ = _camera_list[0].frame.rows, w_ = _camera_list[0].frame.cols;
        Mat frame = Mat(h_, w_ * 2,CV_8UC3);
        Rect rectL(0,0,w_,h_);
        Rect rectR(w_,0,w_,h_);

        _camera_list[0].frame.copyTo(frame(rectL));
        _camera_list[1].frame.copyTo(frame(rectR));
        imshow("Camera System",frame);
    }
}

void Capture::_readNext(){
    if(_conf.main_camera_index <= -1)
        return;
    for(auto& cam:_camera_list){
        string path = cam.primary ? "/next/primary.txt" : "/next/secondary.txt";
        cam.readNext((_conf.storage_path + path).c_str());
    }
}

void Capture::_drawNext(){
    if(_conf.main_camera_index <= -1)
        return;
    thread t1([&](){    _camera_list[0].drawNext(); });
    if(_conf.second_camera_index >= 0){
        thread t2([&](){    _camera_list[1].drawNext(); });
        t2.join();
    }
    t1.join();
}

bool Capture::_chessboardDetect(){
    bool found_main, found_second = true;

    thread t1([&](){    found_main = _camera_list[0].chessboardDetect();    });
    // secondary camera
    if(_conf.second_camera_index >= -1)
        thread([&](){   found_second = _camera_list[1].chessboardDetect();  }).join();

    t1.join();
    return found_main & found_second;
}

bool Capture::_autoCornerMatch(){
    bool match_main, match_second = true;
    thread t1([&](){    match_main = _camera_list[0].cornerMatch(_conf.auto_capture_range); });
    // secondary camera
    if(_conf.second_camera_index >= -1)
        thread ([&](){match_second = _camera_list[1].cornerMatch(_conf.auto_capture_range);}).join();

    t1.join();
    return match_main & match_second;
}

int Capture::captureImageRT(Mat* frame_main,Mat* frame_second){
    captureFrame();

    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);
    if(_conf.second_camera_index >= 0 && frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
    return 0;
}

int Capture::importImage(Mat* frame_main,Mat* frame_second){
    readCountMatch();
    if(count_match == 0)
        return 1;

    _count_cyc = _count_cyc % count_match + 1;
    for(auto& cam : _camera_list){
        string tag = cam.primary ? "primary(" : "secondary(";
        string path = "/match/" + tag + to_string(_count_cyc) + ").jpg";
        cam.importFrame((_conf.storage_path + path).c_str());
    }
    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);
    if(_conf.second_camera_index >= -1 && frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
    return 0;
}

void Capture::captureFrame(){
    for(auto& cam : _camera_list)   
        cam.captureFrame();
    
    if(_conf.dual_sync){
        Camera* cam1 = &_camera_list[0], *cam2 = &_camera_list[1];
        cam2->frame = cam1->frame(Range(0,_conf.image_size.height),Range(_conf.image_size.width, _conf.image_size.width * 2));
        cam1->frame = cam1->frame(Range(0,_conf.image_size.height),Range(0,_conf.image_size.width));
    }

    for(auto& cam : _camera_list)
        cam.frame.copyTo(cam.frame_bak);
    
}

int Capture::captureImage(int flag,Mat* frame_main,Mat* frame_second){
    if(_camera_list[0].index == -1){
        if(flag & MATCH_MODE)   return importImage(frame_main, frame_second);
        else                    return 1;
    }

    if(flag & MATCH_MODE)
        readCountMatch();
        
    int key;
    bool available = false, match = false;
    if(flag & INIT_CAP)
        writeCount(0);
    if(flag & NEXTPOSE)
        _readNext();
    
    for(;;){
        captureFrame();

        key = waitKey(5);
        if(key == KEY_ESC){
            logging.info("capture cancelled\n");
            break;
        }

        if(flag & NEXTPOSE) _drawNext();

        available = ((flag & DETECT) ? _chessboardDetect() : true);

        if((flag & NEXTPOSE) && available && (_conf.auto_capture_range > 0))
            match = _autoCornerMatch();

        if(available && (key == KEY_SPACE || match)){
            if(flag & MATCH_MODE)   writeCountMatch(++count_match);
            else                    writeCount(++count);
            storeImage(flag,frame_main,frame_second);
            
            logging.info("successfully captured image with index %d\n",count);
            if(!(flag & MULTI_CAP)){
                key = KEY_SPACE;
                break;
            }
        } 

        if(!(flag & NOT_SHOW))  _showFrame();
    }
    if(!(flag & NOT_CLOSE)) cv::destroyWindow("Camera System");


    if(key == KEY_SPACE)
        return 0;
    return 1;
}


void Capture::storeImage(int flag, Mat* frame_main,Mat* frame_second){
    string fold = (flag & MATCH_MODE) ?  "/match" : "/images";
    int count_ = (flag & MATCH_MODE) ?  count_match : count; 
    
    if(flag & SAVE_IMAGE){
        string filepath = _conf.storage_path + fold + "/primary(" + to_string(count_) + ").jpg";
        imwrite(filepath, _camera_list[0].frame_bak);
    }
    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);

    // for secondary camera
    if(_conf.second_camera_index < -1)  return;
    if(flag & SAVE_IMAGE){
        string filepath = _conf.storage_path + fold + "/secondary(" + to_string(count_) + ").jpg";
        imwrite(filepath, _camera_list[1].frame_bak);
    }
    if(frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
   
}


int Capture::readCount(){
    ifstream fin(_conf.storage_path + "/images/count.txt", ios::in);
    if(!fin.is_open())  count = 0;
    else                fin >> count;
    if(!fin.eof())      fin >> count_free;
    fin.close();
    return count;
}

int Capture::limitCount(){
    readCount();
    return count - count_free;
}

void Capture::writeCount(int count_input){
    if(count_input >= 0) count = count_input;
    ofstream fout(_conf.storage_path +  "/images/count.txt", ios::out);
    fout << count;
    fout.close();
}

int Capture::readCountMatch(){
    ifstream fin(_conf.storage_path + "/match/count.txt", ios::in);
    if(!fin.is_open())  count_match = 0;
    else                fin >> count_match;
    fin.close();
    return count_match;
}

void Capture::writeCountMatch(int count_input){
    if(count_input >= 0) count_match = count_input;
    ofstream fout(_conf.storage_path +  "/match/count.txt", ios::out);
    fout << count_match;
    fout.close();
}