#include "function.hpp"

int measure(int mode){
    logging.info("measure function\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    calib.importData(0);
    Measure ms(conf);
    ms.readPoints();
    
    cap.openCamera();
    ms.init(mode);

    int key = 0;
    for(;key != KEY_ESC;){
        int res = cap.captureImage(cap.SAVE_IMAGE | cap.MATCH_MODE, &ms._frameL, &ms._frameR);
        if(res == 1)
            break;
        calib.stereoRectify(ms._frameL, ms._frameR);
        key = 0;
        ms.showEpi();

        if(cv::waitKey() == KEY_ESC)    break;
        
        key = 0;
        if(mode == Measure::ADCensus)
            ms.compute(calib.sdata.Q_mat);

        for(;key != KEY_SPACE && key != KEY_ESC;){
            key = cv::waitKey(5);
            if(mode != Measure::ADCensus)
                ms.compute(calib.sdata.Q_mat);
                
            ms.drawDist();
            ms.showMeasure();
        }
        cv::destroyWindow("Measurement");
    }
    cv::destroyAllWindows();
    return 0;
}


/**
 * @brief real-time measurement
 * 
 * @param mode disparity mode
 * @return int 
 */
int measureRT(int mode){
    if(mode == Measure::ADCensus)
        logging.critical(-1,"Real-Time mode isn't supported with AD-Census algorithm");
    
    logging.info("real-time measure function\n");

    Config conf("../config.xml");
    Capture cap(conf);
    Calibrate calib(conf);
    calib.importData(0);

    Measure ms(conf);
    ms.readPoints();
    ms.init(mode);
    cap.openCamera();

    int key = 0;
    for(;key != KEY_ESC;){
        key = cv::waitKey(5);
        cap.captureImageRT(&ms._frameL,&ms._frameR);
        calib.stereoRectify(ms._frameL, ms._frameR);

        ms.compute(calib.sdata.Q_mat);
        ms.drawDist();
        ms.showMeasure();
    }
    cv::destroyAllWindows();
    return 0;
}