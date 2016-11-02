#include <stdio.h>
#include "camera.h"
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerCamera, "Camera");

#ifdef ISTRO_OPENCV_CAMERA

int Camera::init(void) 
{
    cap0.open(0);
    
    if (!cap0.isOpened()) {
        LOGM_ERROR(loggerCamera, "init", "Could not initialize camera[0]!");
        return -1;
    }

#ifdef ISTRO_GUI        
    namedWindow( "Camera0", 0 ); 

    Mat img;
    img.create(640,480,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    imshow("Camera0", img);
#endif

    return 0;
}

void Camera::close(void)
{
}

int Camera::getFrame(Mat& frame)
{
    cap0 >> frame;
	if( frame.empty() )
	    return -1;
  
    return 0;
} 

#else

Mat camera_img0;
    
int Camera::init(void) { 
    LOGM_WARN(loggerCamera, "init", "MOCK CAMERA implementation, DEBUG ONLY!!");

    camera_img0 = imread("sample/0001032_cesta_oblacno.jpg", 1);
    if (!camera_img0.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file!");
        return -1;
    }

#ifdef ISTRO_GUI        
    namedWindow( "Camera0", 0 ); 

    Mat img;
    img.create(640,480,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    imshow("Camera0", img);
#endif

    return 0; 
}

void Camera::close(void) 
{ 
} 

int Camera::getFrame(Mat& frame) 
{ 
    msleep(100);
    frame = camera_img0;
    return 0; 
}

#endif

void Camera::drawFrame(const Mat& frame)
{
#ifdef ISTRO_GUI    
    imshow("Camera0", frame);
#endif    
}
