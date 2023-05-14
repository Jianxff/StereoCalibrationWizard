#include "function.hpp"

int initCalibrate(){
    logging.info("initialize calibration\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    Timer tm;
    cap.writeCount(0);
    calib.clearData();

    cap.openCamera();
    tm.start();
    int res = cap.captureImage(cap.INIT_CAP | cap.MULTI_CAP | cap.DETECT | cap.SAVE_IMAGE);
    tm.end();
    double duration = tm.durationSec();
    logging.info("capture %d images in %f seconds\n", cap.count, duration);
    if(cap.readCount() <= 0)
        return 1;    

    tm.start();
    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    tm.end();
    duration = tm.durationSec();
    logging.info("calibration complete in %f seconds\n", duration);
    calib.pushbackTimer(duration);
    calib.storeData();
    cv::destroyAllWindows();
    
    return res;
}

int nextCalibrate(){
    logging.info("calibration for nextpose\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    Timer tm;

    calib.importData(cap.limitCount());

    cap.openCamera();
    tm.start();
    int res = cap.captureImage(cap.NEXTPOSE | cap.DETECT | cap.SAVE_IMAGE);
    tm.end();
    double duration = tm.durationSec();
    logging.info("capture next images in %f seconds\n", duration);
    cap.readCount();   

    if(res == 1)
        calib.rollBack();
    
    tm.start();
    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    tm.end();
    duration = tm.durationSec();
    logging.info("calibration complete in %f seconds\n", duration);
    calib.pushbackTimer(duration);

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
    Timer tm;

    calib.importData(cap.limitCount());

    tm.start();
    calib.readImage(cap.count);
    calib.cameraCalibrate();
    calib.stereoCalibrate();
    tm.end();
    double duration = tm.durationSec();
    logging.info("calibration complete in %f seconds\n", duration);
    calib.pushbackTimer(duration);
    cap.writeCount();
    calib.storeData();

    logging.debug("calibrate for exsited images complete\n");

    return 0;
}

int freeCalibrate(int total){
    logging.info("start free capture\n");
    // initCalibrate();
    
    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    Timer tm;

    if(conf.free_cap_num > 0)
        total = conf.free_cap_num;

    int cur = cap.readCount();
    int rest = total - cur;
    logging.info("%d images to be captured\n",rest);
    calib.importData(1);

    cap.openCamera();
    int res = 0;
    tm.start();
    for(int i = 0; i < rest && res == 0; i++){
        cap.readCount();
        res = cap.captureImage(cap.DETECT | cap.NOT_CLOSE | cap.SAVE_IMAGE);
    }
    tm.end();
    cv::destroyAllWindows();
    double duration = tm.durationSec();
    logging.info("capture %d images in %f seconds\n", cap.count - cur, duration);


    for(int i = cur; i < total; i++){
        tm.start();
        calib.readImage(i + 1);
        calib.cameraCalibrate();
        calib.stereoCalibrate();
        tm.end();
        duration = tm.durationSec();
        logging.info("calibration complete in %f seconds\n", duration);
        calib.pushbackTimer(duration);
    }
    calib.storeData();
    logging.debug("free calibrate complete\n");

    return 0;
}