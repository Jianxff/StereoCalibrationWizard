#include "capture.hpp"
using namespace std;
using namespace cv;

Capture::Capture(){_count_cyc = 0;}

Capture::Capture(Config& c){
    this->_conf = c;
    this->width = c.image_size.width;
    this->height = c.image_size.height;
    _count_cyc = 0;

    _camera_list.emplace_back(Camera(_conf.main_camera_index));
    if(_conf.second_camera_index >= -1)
        _camera_list.emplace_back(Camera(_conf.second_camera_index, false));
}

void Capture::openCamera(){
    if(_conf.main_camera_index <= -1 || _conf.second_camera_index <= -1)
        return;
    
    for(auto& cam : _camera_list){
        string tag = cam.primary ? "primary" : "secondary";
        if(!cam.init(_conf.image_size, _conf.board_size))
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
        Mat frame = Mat(height, 2 * width,CV_8UC3);
        Rect rectL(0,0,width,height);
        Rect rectR(width,0,width,height);
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
    for(auto& cam : _camera_list)   cam.captureFrame();
    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);
    if(_conf.second_camera_index >= 0 && frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
    return 0;
}

int Capture::importImage(Mat* frame_main,Mat* frame_second){
    readCount(true);
    if(count == 0)
        logging.critical(-1,"no input images\n");

    _count_cyc = _count_cyc % count + 1;
    for(auto& cam : _camera_list){
        string tag = cam.primary ? "primary(" : "secondary(";
        string path = "/input/" + tag + to_string(_count_cyc) + ").jpg";
        cam.importFrame((_conf.storage_path + path).c_str());
    }
    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);
    if(_conf.second_camera_index >= -1 && frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
}

int Capture::captureImage(int flag,Mat* frame_main,Mat* frame_second){
    if(_camera_list[0].index == -1)
        return importImage(frame_main, frame_second);


    int key;
    bool available = false, match = false;
    if(flag & INIT_CAP)
        writeCount(0);
    if(flag & NEXTPOSE)
        _readNext();
    
    for(;;){
        for(auto& cam : _camera_list)   cam.captureFrame();

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
            count++;
            storeImage(flag,frame_main,frame_second);
            if(flag & MEASURE_MODE)
                writeCount(-1,true);
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
    if(flag & SAVE_IMAGE){
        string fold = (flag & MEASURE_MODE) ?  "/input" : "/images";
        string filepath = _conf.storage_path + fold + "/primary(" + to_string(count) + ").jpg";
        imwrite(filepath, _camera_list[0].frame_bak);
    }
    if(frame_main != nullptr)   _camera_list[0].frame_bak.copyTo(*frame_main);

    // for secondary camera
    if(_conf.second_camera_index < -1)  return;
    if(flag & SAVE_IMAGE){
        string fold = (flag & MEASURE_MODE) ?  "/input" : "/images";
        string filepath = _conf.storage_path + fold + "/secondary(" + to_string(count) + ").jpg";
        imwrite(filepath, _camera_list[1].frame_bak);
    }
    if(frame_second != nullptr) _camera_list[1].frame_bak.copyTo(*frame_second);
   
}


int Capture::readCount(bool measure_mode){
    string fold = measure_mode ? "/input/count.txt" : "/images/count.txt";
    ifstream fin(_conf.storage_path + fold, ios::in);
    if(!fin.is_open())  count = 0;
    else                fin >> count;
    fin.close();
    return count;
}

void Capture::writeCount(int count_input, bool measure_mode){
    string fold = measure_mode ? "/input/count.txt" : "/images/count.txt";
    if(count_input >= 0) count = count_input;
    ofstream fout(_conf.storage_path + fold, ios::out);
    fout << count;
    fout.close();
}