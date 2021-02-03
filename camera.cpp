#include <stdio.h>
#include "camera.h"
#include "vision_depth.h"
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerCamera, "Camera");

#ifndef WIN32
const string DEPTH_DATA_FNAME = "out/camera_depth.json";
#else
const string DEPTH_DATA_FNAME = "out\\camera_depth.json";
#endif

#ifdef ISTRO_CAMERA_REALSENSE

int Camera::init(void) 
{
    //Contruct a pipeline which abstracts the device
    //rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, RS2_FORMAT_BGR8, 15);  // 30);
#ifdef ISTRO_CAMERA_RS_DEPTH
    cfg.enable_stream(RS2_STREAM_DEPTH, CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT, RS2_FORMAT_Z16, 15);  // 30);
#endif

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
    
//    if (!cap0.isOpened()) {
//        LOGM_ERROR(loggerCamera, "init", "Could not initialize camera[0]!");
//        return -1;
//    }

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
    //Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();

    //Get each frame
    rs2::frame color_frame = frames.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat color_mat(Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    resize(color_mat, color_mat, Size(640, 480), 0, 0, INTER_NEAREST);  // INTER_AREA is slow (70ms)

    color_mat.copyTo(frame);
    if( frame.empty() )
        return -1;
  
    return 0;
} 

int Camera::getFrameDepth(Mat& frame)
{
#ifdef ISTRO_CAMERA_RS_DEPTH
    //rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
    //Mat depth_mat(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

    rs2::frame depth_frame = frames.get_depth_frame();
    Mat depth_mat(Size(CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    
    // rescale to an unsupported resolution?
    // resize(depth_mat, depth_mat, Size(CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT), 0, 0, INTER_NEAREST);

    depth_mat.copyTo(frame);
    if (!frame.empty()) {
        return 0;
    }
#else

#ifdef ISTRO_VISION_DEPTH
    VisionDepth::getTestData(frame);
    return 0;
#else
    if (!frame.empty()) {
        frame.release();
    }
#endif

#endif

    return -1;
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

int Camera::getFrameDepth(Mat& frame)
{
#ifdef ISTRO_VISION_DEPTH
    VisionDepth::getTestData(frame);
    return 0;
#else
    if (!frame.empty()) {
        frame.release();
    }
#endif
    return -1;
} 

#endif

void Camera::drawFrame(const Mat& frame)
{
#ifdef ISTRO_GUI    
    imshow("Camera0", frame);
#endif    
}

/*
{"CAMERA_DEPTH_FILE":[
{"image_number":"0001527","cdepth_data":[
{"row":4,"d":[11,2600,13]},
{"row":12,"d":[11,2600,13]},
{"col":1,"d":[11,2600,13]}]},
{"image_number":"0001528","cdepth_data":[
{"row":1,"d":[11,2600,0]},
{"row":8,"d":[11,2600,0]},
{"row":20,"d":[11,2600,0]}]}
]}
*/

void Camera::drawDepthFrame(const Mat& frame, Mat &image, const long image_number)
{
    FILE * pFile;
    pFile = fopen(DEPTH_DATA_FNAME.c_str(), "a");
    if (pFile!=NULL) {
        fprintf(pFile, "{\"image_number\":%d,\"cdepth_data\":[\n", (int)image_number);

        int sep = 0;
        int dy = frame.rows / 5;
        int dx = frame.cols / 4;

        for (int y = 0; y < frame.rows; y++) {
            if ((y == 0) || (y % dy) != 0) continue;

            if (sep) fputs(",\n", pFile);
            fprintf(pFile, "{\"row\":%d,\"d\":[", y);
            for (int x = 0; x < frame.cols - 1; x++) {
                fprintf(pFile, "%d,", (int)frame.at<ushort>(y, x));
            }
            int x = frame.cols - 1;
            fprintf(pFile, "%d]}", (int)frame.at<ushort>(y, x));
            sep = 1;
        }

        for (int x = 0; x < frame.cols; x++) {
//            if ((x % 16) != 7) continue;
            if ((x == 0) || (x % dx) != 0) continue;

            if (sep) fputs(",\n", pFile);
            fprintf(pFile, "{\"col\":%d,\"d\":[", x);
            for (int y = 0; y < frame.rows - 1; y++) {
                fprintf(pFile, "%d,", (int)frame.at<ushort>(y, x));
            }
            int y = frame.rows - 1;
            fprintf(pFile, "%d]}", (int)frame.at<ushort>(y, x));
            sep = 1;
        }

        if (sep) fputs("]},\n", pFile);
        fclose(pFile);
    }

    frame.convertTo(image, CV_8U, 1 / 256.0);
    equalizeHist(image, image);
    applyColorMap(image, image, COLORMAP_RAINBOW);  //COLORMAP_JET);
    
    //if ((CAMERA_FRAME_WIDTH != CAMERA_DEPTH_FRAME_WIDTH) || (CAMERA_FRAME_HEIGHT != CAMERA_DEPTH_FRAME_HEIGHT)) {
    //    resize(image, image, Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT));
    //}
}
