#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <opencv2/opencv.hpp>
#include "system.h"

using namespace cv;
using namespace std;

#ifdef ISTRO_RPLIDAR

#include "rplidar.h"    //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

#endif
    
using namespace cv;
using namespace std;

const int LIDAR_DATA_NUM = 360 * 2;

typedef struct { 
    int   sync;
    float angle;
    float distance;
    int   quality;
} lidar_data_t;


class Lidar {
public:    
#ifdef ISTRO_RPLIDAR    
    RPlidarDriver * drv;
#endif

public:  // output
//  int lidar_data_cnt;
//  lidar_data_t lidar_data[LIDAR_DATA_NUM];

//  int lidar_sint; 
float lidar_lobst_fi; 
float lidar_robst_fi;
float lidar_stear_vect;

//  Mat lidar_img;

public:
    int init(const char *portName);
    void close(void);
    
    bool checkHealth(void);

    int getData(lidar_data_t *data, int& data_cnt);
    int process(const lidar_data_t *data, const int& data_cnt, int *dmap, int &stop);
    int drawOutput(const lidar_data_t *data, const int& data_cnt, Mat& img, const int *dmap, int stop, int angle_min, int angle_max, int process_angle, int process_angle_min, int process_angle_max, long image_number);
};

#endif
