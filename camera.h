#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "system.h"

#ifdef ISTRO_CAMERA_REALSENSE
#include <librealsense2/rs.hpp>
#endif

using namespace cv;
using namespace std;

const int CAMERA_FRAME_WIDTH         = 848;
const int CAMERA_FRAME_HEIGHT        = 480;

/* 
https://support.intelrealsense.com/hc/en-us/community/posts/360039243533-Resolution-Configuration-Options-on-Real-Sense-D435-for-Raspberry-Pi-4-4-GB-RAM
The RealSense SDK has a tool called rs-enumerate-devices that can list 
the supported modes on the specific hardware that it is run on.
  16:9  ->   640x360, 480x270, 424x240 @ 6/15/30/60/90 FPS, ... 1280x720 @ 6/15/30 FPS, 
   4:3  ->   640x480 @ 6/15/30/60/90 FPS 
*/

const int CAMERA_DEPTH_FRAME_WIDTH   = 848;
const int CAMERA_DEPTH_FRAME_HEIGHT  = 480;

class Camera {
public:
#ifdef ISTRO_CAMERA_REALSENSE
    rs2::pipeline pipe;
    rs2::frameset frames;
    rs2::colorizer color_map;
#endif
    
public:
    int init(void);
    void close(void);
    
    int getFrame(Mat& frame);
    int getFrameDepth(Mat& frame);
    void drawFrame(const Mat& frame);
    void drawDepthFrame(const Mat& frame, Mat &image, const long image_number);
};

#endif
