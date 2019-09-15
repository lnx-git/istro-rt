#include <stdio.h>
#include "camera.h"
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerCamera, "Camera");

#ifdef ISTRO_OPENCV_CAMERA

int Camera::init(void) 
{
    cap0.open(-1);
    
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
Mat camera_img1;    
Mat camera_img2;    
Mat camera_img3;    
Mat camera_img4;    
Mat camera_img5;

int Camera::init(void) { 
    LOGM_WARN(loggerCamera, "init", "MOCK CAMERA implementation, DEBUG ONLY!!");

    //camera_img0 = imread("sample/0001032_cesta_oblacno.jpg", 1);
    //camera_img0 = imread("sample/0001060_ok.jpg", 1);  // no obstacles
    camera_img0 = imread("sample/0001126_cesta_vpravo.jpg", 1);  // obstacle on the left side
    if (!camera_img0.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #1!");
        return -1;
    }

/*
    camera_img1 = imread("sample/zbar-test.jpg", 1);  // QR-code test
    if (!camera_img1.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #2!");
        return -2;
    }

    camera_img2 = imread("sample/0034400_fisheye.jpg", 1);  // no obstacle
//    camera_img2 = imread("sample/0001060_ok.jpg", 1);  // no obstacle
    if (!camera_img2.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #3!");
        return -3;
    }
*/
/*
    camera_img1 = imread("sample/speetctl_test/0002200_camera.jpg");
    camera_img2 = imread("sample/speetctl_test/0002250_camera.jpg");
    camera_img3 = imread("sample/speetctl_test/0002302_camera.jpg");
    camera_img4 = imread("sample/speetctl_test/0002339_camera.jpg");
    camera_img5 = imread("sample/speetctl_test/0005022_camera.jpg");
*/
    camera_img1 = camera_img0;
    camera_img2 = imread("sample/2014689_kuzele.jpg");
    camera_img3 = imread("sample/2013222_kuzele.jpg");
    camera_img4 = imread("sample/kuzele_test/0031677_camera.jpg");
    camera_img5 = imread("sample/2008241_kuzele.jpg"); 

    if ((!camera_img1.data) || (!camera_img2.data) || (!camera_img3.data) || (!camera_img4.data) || (!camera_img5.data)) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #2-6!");
        return -3;
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

int getframe_cnt = 0;

int Camera::getFrame(Mat& frame) 
{ 
    msleep(100);
    if (getframe_cnt < 50) {
        frame = camera_img1;  // camera_img0;
    } else 
    if (getframe_cnt < 100) {
        frame = camera_img2;
    } else
    if (getframe_cnt < 150) {
        frame = camera_img3;
    } else 
    if (getframe_cnt < 200) {
        frame = camera_img4;
    } else {
        frame = camera_img5;
    }

    getframe_cnt++;
    return 0; 
}

#endif

void Camera::drawFrame(const Mat& frame)
{
#ifdef ISTRO_GUI    
    imshow("Camera0", frame);
#endif    
}
