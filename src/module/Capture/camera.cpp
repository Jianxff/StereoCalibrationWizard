#include "capture.hpp"
using namespace std;
using namespace cv;

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "quartz.lib")

Camera::Camera(int index,bool primary){
    this->index = index;
    this->primary = primary;
    cap = nullptr;
    frame = Mat();
}

bool Camera::init(Config& c){
    _conf = c;
    if(c.board_size.width > 0 && c.board_size.height > 0){
        corner_index[0] = 0;
        corner_index[1] = c.board_size.width - 1;
        corner_index[2] = c.board_size.width * (c.board_size.height - 1);
        corner_index[3] = c.board_size.width * c.board_size.height - 1;
    }
    
    if(index == -1)
        return false;
    if(!primary && c.dual_sync){
        sync = true;
        return true;
    }
    
    cap = new VideoCapture(index);

    int width = c.dual_sync ? c.image_size.width * 2 : c.image_size.width;
    cap->set(CAP_PROP_FRAME_WIDTH, width);
    cap->set(CAP_PROP_FRAME_HEIGHT,c.image_size.height);
    cap->set(CAP_PROP_FPS,60);
    // cap->get(CAP_PROP_FOURCC);
    if(c.mjpg)  cap->set(CAP_PROP_FOURCC,VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // else        cap->set(CAP_PROP_FOURCC,VideoWriter::fourcc('Y', 'U', 'Y', '2'));

    if(c.manual)
        cap->set(CAP_PROP_SETTINGS,1);

    int cc = (int)cap->get(CAP_PROP_FOURCC);
    logging.info("video 4cc '%c%c%c%c'\n",cc&0xFF, (cc>>8)&0xFF, (cc>>16)&0xFF, (cc>>24)&0xFF);

    return cap->isOpened();
}


void Camera::captureFrame(){
    if(cap == nullptr)
        return;
    (*cap) >> frame;
    
    // cvtColor(frame,frame,CV_YCrCb2BGR);
    // frame.copyTo(frame_bak);
}

void Camera::importFrame(const char* filepath){
    frame = cv::imread(filepath);
    frame.copyTo(frame_bak);
}

bool Camera::chessboardDetect(){
    bool found = cv::findChessboardCorners(frame_bak,_conf.board_size,_chessboard_buf,
                                            cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_ADAPTIVE_THRESH |
                                            cv::CALIB_CB_FILTER_QUADS);
    drawChessboardCorners(frame,_conf.board_size,_chessboard_buf,found);
    if(found){
        Mat frame_gray;
        cvtColor(frame_bak,frame_gray,COLOR_RGB2GRAY);
        cornerSubPix(frame_gray,_chessboard_buf, Size(5,5),Size(-1,-1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
        for(int i = 0; i < 4; i++){
            _chessboard_corners[i] = _chessboard_buf[corner_index[i]];
            cv::circle(frame, _chessboard_corners[i], 4, Scalar(255,255,255), -1, 8);
        }
    }
    return found;
}


void Camera::readNext(const char* filepath){
    ifstream fin(filepath,ios::in);
    if(!fin.is_open())
        logging.critical(-1,"next-pose file not found.\n");
    float x,y;
    while(!fin.eof()){
        fin >> x >> y;
        _nextpose_buf.emplace_back(Point2f(x,y));
    }
    fin.close();

    for(int i = 0; i < 4; i++)
        _nextpose_corners[i] = _nextpose_buf[corner_index[i]];
    sort(_nextpose_corners,_nextpose_corners + 4, [&](Point2f& v1,Point2f& v2){ 
        return v1.x < v2.x;
    });
}



void Camera::drawNext(){
    Point2f pre,cur = _nextpose_buf[0];
    circle(frame, cur, 3, Scalar(0,0,255),-1,8);
    for(int i = 1; i < _nextpose_buf.size(); i++){
        pre = cur;
        cur = _nextpose_buf[i];
        circle(frame, cur, 3, Scalar(0,0,255), -1, 8);          // draw points 
        line(frame, pre, cur, Scalar(255, 255, 255), 2, 8);     // draw lines between circles
    }

    for(int i = 0; i < 4; i++)                                  // high light four corners 
        circle(frame, _nextpose_buf[corner_index[i]], 4, Scalar(105, 105, 105), -1, 8);

}

bool Camera::cornerMatch(double range){
    sort(_chessboard_corners,_chessboard_corners + 4, [&](Point2f& v1,Point2f& v2){ 
        return v1.x < v2.x;
    });
    bool match = true;
    for(int i = 0; i < 4 && match; i++){
        Point2f p1 = _chessboard_corners[i], p2 = _nextpose_corners[i];
        double dist = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
        if(dist > range)    match = false;
    }
    return match;
}



void Capture::listCamera(){
    vector<string> list;
    int count = _listDevice(list);
    logging.info("%d devices detected:\n",count);
    for(int i = 0; i < list.size(); i++)
        logging.info("  index: %d, model: %s\n",i,list[i].c_str());
}


/**
 * @brief list connected devices
 * 
 * @param list 
 * @return int number of devices
 */
int Capture::_listDevice(vector<string>& list){
    ICreateDevEnum *pDevEnum = NULL;
    IEnumMoniker *pEnum = NULL;
    int deviceCounter = 0;
    CoInitialize(NULL);
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
        CLSCTX_INPROC_SERVER, IID_ICreateDevEnum,
        reinterpret_cast<void**>(&pDevEnum));
    if (SUCCEEDED(hr)){
        // Create an enumerator for the video capture category.
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
        if (hr == S_OK){
            IMoniker *pMoniker = NULL;
            while (pEnum->Next(1, &pMoniker, NULL) == S_OK){
                IPropertyBag *pPropBag;
                hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag,
                    (void**)(&pPropBag));
                if (FAILED(hr)){
                    pMoniker->Release();
                    continue;  // Skip this one, maybe the next one will work.
                }
                // Find the description or friendly name.
                VARIANT varName;
                VariantInit(&varName);
                hr = pPropBag->Read(L"Description", &varName, 0);
                if (FAILED(hr)) hr = pPropBag->Read(L"FriendlyName", &varName, 0);
                if (SUCCEEDED(hr)){
                    hr = pPropBag->Read(L"FriendlyName", &varName, 0);
                    int count = 0;
                    char tmp[255] = {0};
                    while (varName.bstrVal[count] != 0x00 && count < 255) {
                        tmp[count] = (char)varName.bstrVal[count];
                        count++;
                    }
                    list.push_back(tmp);
                }
                pPropBag->Release();
                pPropBag = nullptr;
                pMoniker->Release();
                pMoniker = nullptr;
                deviceCounter++;
            }
            pDevEnum->Release();
            pDevEnum = nullptr;
            pEnum->Release();
            pEnum = nullptr;
        }
    }
    return deviceCounter;
}