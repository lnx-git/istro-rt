#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "lidar.h"
#include "ctrlboard.h"
#include "system.h"
#include "sample.h"
#include "logger.h"

const float LIDAR_DISTANCE_MIN       =   1;    // minimum distance that could be measured by lidar (in centimeters) 
const float LIDAR_DISTANCE_THRESHOLD = 150;    // obstacle avoidance distance (in centimeters)
const int   LIDAR_QUALITY_THRESHOLD  =   1;    // ignore datected obstacle if quality is low (<20)

const float LIDAR_STOP_ANGLE_MIN     =  45;    // minimum angle where distance is checked
const float LIDAR_STOP_ANGLE_MAX     = 135;    // maximum angle where distance is checked
const float LIDAR_STOP_DISTANCE      =  50;    // minimum distance that will stop the robot (in centimeters) 
const int   LIDAR_STOP_COUNT         =   3;    // number of angles where the distance must be exceeded

#ifndef WIN32
const string outputFName = "out/lidar.log";
#else
const string outputFName = "out\\lidar.log";
#endif

LOG_DEFINE(loggerLidar, "Lidar");

#ifdef ISTRO_RPLIDAR

int Lidar::init(const char *portName)
{
    const char * opt_com_path = portName;
    _u32         opt_com_baudrate = 115200;

    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        LOGM_ERROR(loggerLidar, "init", "insufficent memory!");
        return -1;
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        LOGM_ERROR(loggerLidar, "init", "cannot bind to serial port \"" << opt_com_path << "\"");
        RPlidarDriver::DisposeDriver(drv);
        return -2;
    }

    // check health...
    if (!checkHealth()) {
        LOGM_ERROR(loggerLidar, "init", "checkHealth failed!");
        RPlidarDriver::DisposeDriver(drv);
        return -3;
    }

    // start scan...
    drv->startScan();

#ifdef ISTRO_GUI    
    namedWindow("Lidar", 0 );
    resizeWindow("Lidar", 640, 480);

    /* fixme: not calling imshow in Lidar::init() sometimes caused segmentation fault in Lidar::drawOutput() */
    Mat img;
    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    imshow("Lidar", img);
#endif

    return 0;
}

bool Lidar::checkHealth(void)
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        LOGM_TRACE(loggerLidar, "checkHealth", "status=" << (int)healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            LOGM_ERROR(loggerLidar, "checkHealth", "internal error detected, please reboot the device!");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        LOGM_ERROR(loggerLidar, "checkHealth", "cannot retrieve health code!, res=" << (int)op_result);
        return false;
    }
}

void Lidar::close(void)
{
    RPlidarDriver::DisposeDriver(drv);
}

int Lidar::getData(lidar_data_t *data, int& data_cnt)
{
    u_result op_result;
    size_t count = LIDAR_DATA_NUM;
    rplidar_response_measurement_node_t nodes[LIDAR_DATA_NUM];
    
    data_cnt = -1;

    op_result = drv->grabScanData(nodes, count);
    if (!IS_OK(op_result)) {
        LOGM_ERROR(loggerLidar, "getData", "grabScanData failed!");
        return -1;
    }
    
    drv->ascendScanData(nodes, count);
    
    for (int pos = 0; pos < (int)count; ++pos) {
        data->sync = nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
        data->angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        data->distance = nodes[pos].distance_q2/4.0f;
        data->quality = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;        
        data++;
    }
    
    data_cnt = (int)count;
    return 0;
}

#else

int Lidar::init(const char *portName) 
{
    LOGM_WARN(loggerLidar, "init", "MOCK LIDAR implementation, DEBUG ONLY!!");
    
#ifdef ISTRO_GUI    
    namedWindow("Lidar", 0 );
    resizeWindow("Lidar", 640, 480);

    /* fixme: not calling imshow in Lidar::init() sometimes caused segmentation fault in Lidar::drawOutput() */
    Mat img;
    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    imshow("Lidar", img);
#endif

    return 0; 
}

bool Lidar::checkHealth(void) 
{ 
    return true; 
}

void Lidar::close(void) 
{ 
}

int Lidar::getData(lidar_data_t *data, int& data_cnt)
{
    int i = 0;

    data[i].sync = 0;  data[i].angle =   0.23;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   1.39;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   2.56;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   3.72;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   4.89;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 1;  data[i].angle =   6.06;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   7.22;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   8.39;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   9.55;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  10.72;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  11.88;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  13.05;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  14.20;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  15.38;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  16.55;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  17.70;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  18.88;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  20.03;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  21.20;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  22.36;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  23.53;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  24.70;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  25.86;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  27.16;  data[i].distance = 00137.50;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  28.31;  data[i].distance = 00147.50;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  29.36;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  30.14;  data[i].distance = 00173.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  31.31;  data[i].distance = 00187.75;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  32.84;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  33.27;  data[i].distance = 00303.25;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  33.94;  data[i].distance = 00440.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  34.02;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  34.34;  data[i].distance = 00556.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  35.19;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  37.52;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  42.38;  data[i].distance = 00134.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  42.44;  data[i].distance = 00126.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  44.41;  data[i].distance = 00144.25;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  44.58;  data[i].distance = 00156.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  45.25;  data[i].distance = 00168.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  46.41;  data[i].distance = 00186.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  47.17;  data[i].distance = 00228.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  47.56;  data[i].distance = 00205.00;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  47.88;  data[i].distance = 00297.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  48.41;  data[i].distance = 00345.75;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  48.86;  data[i].distance = 00257.50;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  48.89;  data[i].distance = 00416.50;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  49.36;  data[i].distance = 00526.50;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  56.16;  data[i].distance = 00129.75;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  58.48;  data[i].distance = 00149.00;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  58.72;  data[i].distance = 00138.75;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  59.64;  data[i].distance = 00161.50;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  60.30;  data[i].distance = 00177.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  61.47;  data[i].distance = 00194.25;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  61.80;  data[i].distance = 00214.00;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  62.80;  data[i].distance = 00273.25;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.20;  data[i].distance = 00318.75;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  63.36;  data[i].distance = 00374.25;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.36;  data[i].distance = 00241.25;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.69;  data[i].distance = 00465.75;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  64.09;  data[i].distance = 00600.00;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  64.58;  data[i].distance = 00888.50;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  65.06;  data[i].distance = 01683.00;  data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  72.67;  data[i].distance = 04191.75;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  73.62;  data[i].distance = 00000.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  73.86;  data[i].distance = 03831.25;  data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  74.80;  data[i].distance = 00000.00;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  75.06;  data[i].distance = 03455.25;  data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  75.95;  data[i].distance = 00000.00;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  76.23;  data[i].distance = 03181.75;  data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  77.12;  data[i].distance = 00000.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  77.45;  data[i].distance = 02916.00;  data[i++].quality = 14;
    data[i].sync = 0;  data[i].angle =  78.28;  data[i].distance = 00000.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  78.62;  data[i].distance = 02689.75;  data[i++].quality = 13;
    data[i].sync = 0;  data[i].angle =  79.45;  data[i].distance = 00000.00;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  79.86;  data[i].distance = 02500.50;  data[i++].quality = 17;
    data[i].sync = 0;  data[i].angle =  81.06;  data[i].distance = 02351.25;  data[i++].quality = 16;
    data[i].sync = 0;  data[i].angle =  82.25;  data[i].distance = 02222.00;  data[i++].quality = 17;
    data[i].sync = 0;  data[i].angle =  83.47;  data[i].distance = 02093.75;  data[i++].quality = 18;
    data[i].sync = 0;  data[i].angle =  84.64;  data[i].distance = 01990.50;  data[i++].quality = 21;
    data[i].sync = 0;  data[i].angle =  85.83;  data[i].distance = 01893.00;  data[i++].quality = 18;
    data[i].sync = 0;  data[i].angle =  87.02;  data[i].distance = 01807.50;  data[i++].quality = 22;
    data[i].sync = 0;  data[i].angle =  88.27;  data[i].distance = 01731.00;  data[i++].quality = 21;
    data[i].sync = 0;  data[i].angle =  89.41;  data[i].distance = 01665.75;  data[i++].quality = 23;
    data[i].sync = 0;  data[i].angle =  90.61;  data[i].distance = 01591.75;  data[i++].quality = 27;
    data[i].sync = 0;  data[i].angle =  91.84;  data[i].distance = 01531.50;  data[i++].quality = 22;
    data[i].sync = 0;  data[i].angle =  93.02;  data[i].distance = 01472.00;  data[i++].quality = 25;
    data[i].sync = 0;  data[i].angle =  94.19;  data[i].distance = 01426.25;  data[i++].quality = 24;
    data[i].sync = 0;  data[i].angle =  95.39;  data[i].distance = 01383.75;  data[i++].quality = 27;
    data[i].sync = 0;  data[i].angle =  96.59;  data[i].distance = 01339.75;  data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle =  97.81;  data[i].distance = 01297.50;  data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle =  98.94;  data[i].distance = 01260.50;  data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle = 100.14;  data[i].distance = 01226.75;  data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 101.39;  data[i].distance = 01196.75;  data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 102.59;  data[i].distance = 01168.50;  data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 103.77;  data[i].distance = 01139.25;  data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 104.91;  data[i].distance = 01111.75;  data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 106.06;  data[i].distance = 01087.75;  data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 107.33;  data[i].distance = 01062.25;  data[i++].quality = 29;
    data[i].sync = 0;  data[i].angle = 108.52;  data[i].distance = 01042.75;  data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 109.69;  data[i].distance = 01022.75;  data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 110.84;  data[i].distance = 01004.25;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 112.11;  data[i].distance = 00984.75;  data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 113.23;  data[i].distance = 00967.00;  data[i++].quality = 33;
    data[i].sync = 0;  data[i].angle = 114.50;  data[i].distance = 00951.75;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 115.69;  data[i].distance = 00935.50;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 116.81;  data[i].distance = 00921.00;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 117.98;  data[i].distance = 00907.75;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 119.14;  data[i].distance = 00894.75;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 120.45;  data[i].distance = 00882.25;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 121.56;  data[i].distance = 00873.25;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 122.70;  data[i].distance = 00861.25;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 124.02;  data[i].distance = 00850.00;  data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 125.14;  data[i].distance = 00840.75;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 126.27;  data[i].distance = 00833.50;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 127.44;  data[i].distance = 00824.50;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 128.66;  data[i].distance = 00816.25;  data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 129.78;  data[i].distance = 00809.25;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 131.03;  data[i].distance = 00803.00;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 132.16;  data[i].distance = 00795.50;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 133.39;  data[i].distance = 00789.25;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 134.50;  data[i].distance = 00785.00;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 135.67;  data[i].distance = 00778.00;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 136.86;  data[i].distance = 00774.00;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 138.17;  data[i].distance = 00769.25;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 139.28;  data[i].distance = 00766.00;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 140.47;  data[i].distance = 00761.00;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 141.77;  data[i].distance = 00757.50;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 142.78;  data[i].distance = 00756.75;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 144.12;  data[i].distance = 00752.25;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 145.17;  data[i].distance = 00748.75;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 146.33;  data[i].distance = 00748.25;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 147.64;  data[i].distance = 00745.75;  data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 148.80;  data[i].distance = 00744.50;  data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 149.92;  data[i].distance = 00744.25;  data[i++].quality = 43;
    data[i].sync = 0;  data[i].angle = 151.14;  data[i].distance = 00744.25;  data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 152.31;  data[i].distance = 00743.25;  data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 153.42;  data[i].distance = 00744.50;  data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 154.47;  data[i].distance = 00744.50;  data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 155.70;  data[i].distance = 00747.00;  data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 156.88;  data[i].distance = 00746.50;  data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 158.02;  data[i].distance = 00748.50;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 159.28;  data[i].distance = 00749.50;  data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 160.39;  data[i].distance = 00752.50;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 161.58;  data[i].distance = 00756.00;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 162.67;  data[i].distance = 00758.25;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 163.72;  data[i].distance = 00760.50;  data[i++].quality = 33;
    data[i].sync = 0;  data[i].angle = 164.92;  data[i].distance = 00764.00;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 166.06;  data[i].distance = 00766.50;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 167.28;  data[i].distance = 00771.50;  data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 168.44;  data[i].distance = 00776.25;  data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 169.55;  data[i].distance = 00781.50;  data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 170.69;  data[i].distance = 00788.75;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 171.81;  data[i].distance = 00793.75;  data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 173.08;  data[i].distance = 00799.50;  data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 174.19;  data[i].distance = 00805.75;  data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 175.36;  data[i].distance = 00812.75;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 176.48;  data[i].distance = 00819.00;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 177.53;  data[i].distance = 00828.25;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 178.67;  data[i].distance = 00836.00;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 179.81;  data[i].distance = 00846.50;  data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 181.03;  data[i].distance = 00856.00;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 182.23;  data[i].distance = 00865.25;  data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 183.28;  data[i].distance = 00875.50;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 184.53;  data[i].distance = 00886.00;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 185.77;  data[i].distance = 00895.50;  data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 186.94;  data[i].distance = 00909.75;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 188.08;  data[i].distance = 00921.00;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 189.16;  data[i].distance = 00936.50;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 190.31;  data[i].distance = 00950.50;  data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 191.53;  data[i].distance = 00964.00;  data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 192.62;  data[i].distance = 00982.50;  data[i++].quality = 29;
    data[i].sync = 0;  data[i].angle = 193.75;  data[i].distance = 01026.00;  data[i++].quality = 14;
    data[i].sync = 0;  data[i].angle = 201.78;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 202.95;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 204.11;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 205.28;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 206.44;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 207.61;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 208.77;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 209.94;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 211.11;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 212.27;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 213.44;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 214.59;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 215.77;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 216.92;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 218.09;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 219.27;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 220.42;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 221.59;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 222.75;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 223.92;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 225.08;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 226.25;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 227.41;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 228.58;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 229.75;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 230.91;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 232.08;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 233.23;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 234.41;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 235.56;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 236.73;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 237.91;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 239.06;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 240.23;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 241.39;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 242.56;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 243.72;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 244.89;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 246.06;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 247.22;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 248.39;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 249.55;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 250.72;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 251.88;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 253.05;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 254.20;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 255.38;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 256.55;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 257.70;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 258.88;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 260.03;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 261.20;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 262.36;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 263.53;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 264.70;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 265.86;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 267.03;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 268.19;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 269.36;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 270.52;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 271.69;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 272.84;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 274.02;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 275.19;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 276.34;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 277.52;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 278.67;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 279.84;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 281.00;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 282.17;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 283.34;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 284.50;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 285.67;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 286.83;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 288.00;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 289.16;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 290.33;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 291.48;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 292.66;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 293.83;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 294.98;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 296.16;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 297.31;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 298.48;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 299.64;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 300.81;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 301.98;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 303.14;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 304.31;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 305.47;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 306.64;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 307.80;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 308.97;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 310.12;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 311.30;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 312.47;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 313.62;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 314.80;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 315.95;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 317.12;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 318.28;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 319.45;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 320.62;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 321.78;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 322.95;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 324.11;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 325.28;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 326.44;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 327.61;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 328.77;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 329.94;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 331.11;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 332.27;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 333.44;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 334.59;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 335.77;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 336.92;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 338.09;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 339.27;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 340.42;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 341.59;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 342.75;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 343.92;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 345.08;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 346.25;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 347.41;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 348.58;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 349.75;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 350.91;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 352.08;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 353.23;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 354.41;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 355.56;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 356.73;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 357.91;  data[i].distance = 00000.00;  data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 359.06;  data[i].distance = 00000.00;  data[i++].quality =  0;
    
    data_cnt = i;

/*    
    // decrease the distance of all values to be below LIDAR_DISTANCE_THRESHOLD
    for (i = 0; i < data_cnt; i++) {
        data[i].distance = data[i].distance * 0.4;
    }

    lidar_data_t data2[LIDAR_DATA_NUM];
    int i2 = 30;    // rotate 45 "degrees"
    for(i = 0; i < LIDAR_DATA_NUM; i++) {
        data2[i].sync = data[i].sync;  
        data2[i].angle = data[i].angle;  
        data2[i].distance = data[i2].distance;  
        data2[i].quality = data[i2].quality;
        if (++i2 >= LIDAR_DATA_NUM) {
            i2 = 0;
        }
    }
    memcpy(data, data2, LIDAR_DATA_NUM * sizeof(lidar_data_t));    
*/

    msleep(30);
    
    return 0;
}

#endif

int Lidar::process(const lidar_data_t *data, const int& data_cnt, int *dmap, int &stop)
{   
    double t = timeBegin();
    
    dmap_init(dmap);
    float dd = 1;
    int stop_cnt = 0;
    for(int i = 0; i < data_cnt; i++) {
        float angle1 = data[i].angle;
        // low quality => no obstacle
        int val = 1;
        if ((data[i].quality >= LIDAR_QUALITY_THRESHOLD) && (data[i].distance >= LIDAR_DISTANCE_MIN * 10)){
            // distance above threshold => no obstacle
            val = (data[i].distance >= LIDAR_DISTANCE_THRESHOLD * 10);
            if ((angle1 >= LIDAR_STOP_ANGLE_MIN) && (angle1 <= LIDAR_STOP_ANGLE_MAX) && (data[i].distance < LIDAR_STOP_DISTANCE * 10)) {
                stop_cnt++;
            }
        }
        // for the last one we use old value of dd
        if (i + 1 < data_cnt) {
            float angle2 = data[i+1].angle;
            dd = (angle2 - angle1) / 2;    
        }
        // in dmap zero degrees is on the right side of the robot (counterclockwise)
        dmap_fill(dmap, 180 - angle1 - dd, 180 - angle1 + dd, val);
    }
    dmap_finish(dmap);
    
    stop = (stop_cnt >= LIDAR_STOP_COUNT);

    /*
    float lobst_fi = 0;
    float robst_fi = 180;

    for (int pos = 0; pos < data_cnt; ++pos) {
        // x: half of the distance in centimeters
        float x = data[pos].distance / 10;
        if (x < 0.1) x=290;
        float fiact = data[pos].angle;
                                       
        if( x < LIDAR_DISTANCE_THRESHOLD ) {
            if( (fiact>0) && (fiact<90) ) {
                lobst_fi = fiact;
            }
            if( (fiact>90) && (fiact<180)) {
                if(fiact<robst_fi) {
                    robst_fi = fiact;
                }
            }
        }
    }

    float stear_vect = lobst_fi + (robst_fi-lobst_fi)/2.0;

    int sint = int(stear_vect) -90;
    sint = -1*sint*50/90 + SA_STRAIGHT;
    if(sint < SA_MIN) 
       sint = SA_MIN;
    if(sint > SA_MAX)
       sint = SA_MAX;

    lidar_sint = sint;
    lidar_lobst_fi = lobst_fi;
    lidar_robst_fi = robst_fi;
    lidar_stear_vect = stear_vect;
    */
    
    timeEnd("Lidar::process", t);

    return 0;
}

int Lidar::drawOutput(const lidar_data_t *data, const int& data_cnt, Mat& img, const int *dmap, int stop, int angle_min, int angle_max, int process_angle, int process_angle_min, int process_angle_max, long image_number)
{
    double t = timeBegin();    
    char outgingData[30] = "";

    FILE * pFile;
    pFile = fopen (outputFName.c_str(), "a");
    if (pFile!=NULL) {    
        for (int pos = 0; pos < data_cnt; ++pos) {
            fprintf(pFile,"%s %07u theta: %03.2f Dist: %08.2f Q: %d\n", 
                (data[pos].sync) ?"S":"R", image_number, data[pos].angle, data[pos].distance, data[pos].quality);
        }
        fclose (pFile);
    }

    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);
    
    const int x=320;
    
    if (process_angle >= 0) {
        float fi = (-process_angle)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,0),1,8,0);        
    }

    if (process_angle_min >= 0) {
        float fi = (-process_angle_min)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,255),1,8,0);
    }
    
    if (process_angle_max >= 0) {
        float fi = (-process_angle_max)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,255),1,8,0);
    }

    for (int pos = 0; pos < data_cnt; ++pos) {
        // mapping 0..3,2 meters to 10..320 pixels
        float x = data[pos].distance / 10;  
        // set minimum drawing length to correctly show color 
        if (x < 10) x = 20;
        if (x > 320) x = 320;
        
        float fiact = data[pos].angle;
        float fi = (180.0+fiact)*M_PI/180.0;
        
        int xn = 320 + x*cos(fi);
        int yn = 320 + x*sin(fi);

        int angle = trunc(180 - fiact);
        if (angle < 0) {
            angle += 360;
        }
        int angle_ok = (angle >= angle_min) && (angle <= angle_max);
        
        int dist_stop = 0;
        if ((data[pos].quality >= LIDAR_QUALITY_THRESHOLD) && (data[pos].distance >= LIDAR_DISTANCE_MIN * 10)) {
            dist_stop = ((fiact >= LIDAR_STOP_ANGLE_MIN) && (fiact <= LIDAR_STOP_ANGLE_MAX) && (data[pos].distance < LIDAR_STOP_DISTANCE * 10));
        }
        
        Scalar color1 = Scalar(200, 200, 200);
        Scalar color2 = Scalar(255, 128,   0);
        if (angle_ok) {
            color1 = Scalar(0, 200, 0);
        }
        
        if ((data[pos].quality >= LIDAR_QUALITY_THRESHOLD) && (data[pos].distance >= LIDAR_DISTANCE_MIN * 10)) {
            // distance above threshold => no obstacle
            if (data[pos].distance < LIDAR_DISTANCE_THRESHOLD * 10) {
                color1 = Scalar(0, 0, 128);
                color2 = Scalar(255, 255, 0);
            } 
        } else {
            if (angle_ok) {
                color1 = Scalar(0, 100, 0);
                color2 = Scalar(0, 100, 0);
            } else {
                color1 = Scalar(150, 150, 150);
                color2 = Scalar(150, 150, 150);
            }
        }
        
        if (dist_stop) {
            color1 = Scalar(250, 0, 250);
        }

        line(img,Point(320,320),Point(xn,yn),color1,1,8,0);
            
        img.at<Vec3b>(yn,xn)[0] = color2.val[0];
        img.at<Vec3b>(yn,xn)[1] = color2.val[1];
        img.at<Vec3b>(yn,xn)[2] = color2.val[2]; 
    }

    sprintf(outgingData,"process_angle: %d", process_angle);  
    putText(img, outgingData, Point(30,60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255,0)); 
    sprintf(outgingData,"process_angle_min: %d", process_angle_min);  
    putText(img, outgingData, Point(30,90), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255,0)); 
    sprintf(outgingData,"process_angle_max: %d", process_angle_max);  
    putText(img, outgingData, Point(30,120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255,0)); 
    sprintf(outgingData,"stop: %d", stop);  
    putText(img, outgingData, Point(30,150), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255,0)); 
    
#ifdef ISTRO_GUI
    LOGM_TRACE(loggerLidar, "drawOutput", "before imshow()...");
    imshow("Lidar", img);
#endif

    timeEnd("Lidar::drawOutput", t);

    return 0;    
}

