#ifndef __CAPTURE_HPP__
#define __CAPTURE_HPP__

#include "../../basic.hpp"
#include "../Config/config.hpp"

/**
 * @brief Camera Operation Unit
 * 
 */
class Camera{
    std::vector<cv::Point2f> _chessboard_buf, _nextpose_buf;         
    cv::Point2f              _chessboard_corners[4], _nextpose_corners[4];
    int corner_index[4]; 
    Config _conf;
    
    

public:
    Camera(int index = -1,bool primary = true);
    bool            init(Config& c);       // initialize
    void            importFrame(const char* filepath);                      // import from file
    void            captureFrame();                                         // capture by video capture

    void            readNext(const char* filepath);                         // read nextpose data
    void            drawNext();                                             // draw nextpose pattern

    bool            chessboardDetect();                                     // detect chessboard
    bool            cornerMatch(double range);                              // matching four corners

    bool primary = true;        // is primary camera
    bool sync = false;          // sync from primary camera;
    int index;                  // camera index
    cv::VideoCapture *cap = nullptr;      // capture pointer
    cv::Mat frame, frame_bak;   // frame save and backup
};



/**
 * @brief Image Capture Uint
 * 
 */
class Capture{
    std::vector<Camera> _camera_list;        // cameras for captrue
    // int     width;
    // int     height;
    Config  _conf;                      // configure file 
    int     _count_cyc;
    void    _readNext();
    void    _showFrame();               // show frames on screen
    void    _drawNext();                // draw next-pose pattern 
    bool    _chessboardDetect();        // find chessboard corners
    bool    _autoCornerMatch();         // automatically match four corners
    int     _listDevice(std::vector<std::string>&);
public:
    enum CAP_FLAG{
        DETECT          = 1 << 0,
        NEXTPOSE        = 1 << 1,
        SAVE_IMAGE      = 1 << 2,
        MULTI_CAP       = 1 << 3,
        INIT_CAP        = 1 << 4,
        NOT_SHOW        = 1 << 5,
        NOT_CLOSE       = 1 << 6,
        MATCH_MODE      = 1 << 7
    };
    int     count;                      // image count
    int     count_match;
    int     count_free = 0;

    Capture();
    Capture(Config&);
    void    listCamera();
    int     readCount();
    int     limitCount();
    void    writeCount(int count = -1);
    int     readCountMatch();
    void    writeCountMatch(int count = -1);

    void    openCamera();
    void    captureFrame();
    int     importImage(cv::Mat* frame_main = nullptr,cv::Mat* frame_second = nullptr);
    int     captureImage(int flag,cv::Mat* frame_main = nullptr,cv::Mat* frame_second = nullptr);
    int     captureImageRT(cv::Mat* frame_main = nullptr,cv::Mat* frame_second = nullptr);
    void    storeImage(int save_flag,cv::Mat* frame_main = nullptr,cv::Mat* frame_second = nullptr);

};

#endif