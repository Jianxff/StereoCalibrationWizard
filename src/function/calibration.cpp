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
    cv::destroyAllWindows();
    
    return res;
}

int nextCalibrate(){
    logging.info("calibration for nextpose\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);

    calib.importData(cap.limitCount());

    cap.openCamera();
    int res = cap.captureImage(cap.NEXTPOSE | cap.DETECT | cap.SAVE_IMAGE);
    cap.readCount();   

    if(res == 1)
        calib.rollBack();
    
    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    cap.writeCount();
    calib.storeData();
    cv::destroyAllWindows();
    
    logging.debug("next calibrate complete\n");

    return res;
}

int inputCalibrate(){
    logging.info("calibrate input images\n");
    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);

    calib.importData(cap.limitCount());
    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    cap.writeCount();
    calib.storeData();

    logging.debug("calibrate for exsited images complete\n");

    return 0;
}

int freeCalibrate(int total){
    logging.info("start free capture\n");
    initCalibrate();
    
    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);

    if(conf.free_cap_num > 0)
        total = conf.free_cap_num;

    int cur = cap.readCount();
    int rest = total - cur;
    logging.info("%d images to be captured\n",rest);
    calib.importData(1);

    cap.openCamera();
    int res = 0;
    for(int i = 0; i < rest && res == 0; i++){
        cap.readCount();
        res = cap.captureImage(cap.DETECT | cap.NOT_CLOSE | cap.SAVE_IMAGE);
    }
    cv::destroyAllWindows();
    
    for(int i = cur; i < total; i++){
        calib.readImage(i + 1);
        calib.cameraCalibrate();
        calib.stereoCalibrate();    
    }
    calib.storeData();
    logging.debug("free calibrate complete\n");

    return 0;
}