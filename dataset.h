#ifndef __DATASET_H__
#define __DATASET_H__

#include "sample.h"
#include "lidar.h"

class DataSet {
public:
    Mat camera_img;
    
public:
    Mat vision_markers;
    Mat vision_markersIM;

    Mat vision_epweigth;
    Mat vision_elweigth;
    
    int vision_dmap[DEGREE_MAP_COUNT];
    int vision_angle_min;
    int vision_angle_max;
    
public:
    int lidar_data_cnt;
    lidar_data_t lidar_data[LIDAR_DATA_NUM];
    
    int lidar_dmap[DEGREE_MAP_COUNT];
    int lidar_angle_min; 
    int lidar_angle_max;
    int lidar_stop;

public:
    int process_dir;  // heading (45..135) - what direction should we go (45 = right, 90 = forward, 135 = left)
    int process_dmap[DEGREE_MAP_COUNT];
    int process_angle_min;
    int process_angle_max;

    // shared    
    int process_angle;
    int process_stop;
    
public:
    // shared
    int gps_fix;
    double gps_latitude;
    double gps_longitude;
    double gps_course;

    double gps_lastp_dist;     // distance to the previous gps fix position
    double gps_lastp_azimuth;  // azimuth to the previous gps fix position
    
    double gps_navp_dist;     // distance to the next navigation point
    double gps_navp_azimuth;  // azimuth to the next navigation point
    
public:
    // shared
    double ahrs_roll; 
    double ahrs_pitch; 
    double ahrs_yaw;

public:
    // shared
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int ctrlb_calib_gyro;
    int ctrlb_calib_accel;
    int ctrlb_calib_mag;
    
public:
    // shared
    long image_number;
    
public:
    DataSet();
};

#endif
