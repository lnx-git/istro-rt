#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "system.h"

using namespace cv;
using namespace std;

class Camera {
public:
#ifdef ISTRO_CAMERA_OPENCV
    VideoCapture cap0;
#endif
    
public:
    int init(void);
    void close(void);
    
    int getFrame(Mat& frame);
    void drawFrame(const Mat& frame);
};

#endif
