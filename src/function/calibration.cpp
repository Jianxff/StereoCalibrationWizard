#include "function.hpp"

int initCalibrate(){
    logging.info("initialize calibration\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    cap.writeCount(0);
    calib.clearData();

    cap.openCamera();
    int res = cap.captureImage(cap.INIT_CAP | cap.MULTI_CAP | cap.DETECT | cap.SAVE_IMAGE);
    if(cap.readCount() <= 0)
        return 1;    

    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    calib.storeData();
    return res;
}

int nextCalibrate(){
    logging.info("calibration for nextpose\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);

    calib.importData();

    cap.openCamera();
    int res = cap.captureImage(cap.NEXTPOSE | cap.DETECT | cap.SAVE_IMAGE);
    cap.readCount();    
    if(res == 0){
        calib.readImage(cap.count);
        calib.cameraCalibrate();
        calib.stereoCalibrate();
        calib.storeData();
    }
    return res;
}

int freeCalibrate(int total){
    logging.info("start free capture\n");
    initCalibrate();
    
    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);

    int cur = cap.readCount();
    int rest = total - cur;
    logging.info("%d images to be captured\n",rest);
    calib.importData(1);

    cap.openCamera();
    for(int i = 0; i < rest; i++){
        cap.readCount();
        cap.captureImage(cap.DETECT | cap.NOT_CLOSE | cap.SAVE_IMAGE);
    }
    cv::destroyAllWindows();
    
    for(int i = cur; i < total; i++){
        calib.readImage(i + 1);
        calib.cameraCalibrate();
        calib.stereoCalibrate();    
    }
    calib.storeData();
    return 0;
}