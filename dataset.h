#ifndef __DATASET_H__
#define __DATASET_H__

#include "mtime.h"
#include "dmap.h"
#include "lidar.h"

const int PROCESS_STATE_NONE         = 0;
const int PROCESS_STATE_CALIBRATION  = 1;
const int PROCESS_STATE_WRONGWAY     = 2;
const int PROCESS_STATE_LOADING      = 3;
const int PROCESS_STATE_UNLOADING    = 4;
const int PROCESS_STATE_NAV_ANGLE    = 5;
const int PROCESS_STATE_MIN_MAX      = 6;

class DataSet {
public:
    Mat camera_img;
    
public:
    Mat vision_markers;
    Mat vision_markersIM;

    Mat vision_epweigth;
    Mat vision_elweigth;
    
    DegreeMap vision_dmap;
    int vision_angle_min;
    int vision_angle_max;
    
public:
    int lidar_data_cnt;
    lidar_data_t lidar_data[LIDAR_DATA_NUM];
    
    DegreeMap lidar_dmap;
    int lidar_angle_min; 
    int lidar_angle_max;
    int lidar_stop;

public:
    int process_dir;  // heading (45..135) - what direction should we go (45 = right, 90 = forward, 135 = left)
    DegreeMap process_dmap;
    int process_angle_min;
    int process_angle_max;

    // shared    
    int process_angle;
    int process_velocity;
    int process_stop;
    int process_state;

    int    process_ref;         // process_ref=1 means that process_x and process_y are valid
    double process_x;           // calculated x position
    double process_y;           // calculated y position
    double process_yaw;         // robot heading
    double process_time;        // time when we calculated the last position
    
public:
    // shared
    double gps_time;            // when the data were received from GPS device

    int    gps_fix;
    double gps_latitude;
    double gps_longitude;
    double gps_speed;
    double gps_course;

    double gps_lastp_dist;      // distance to the previous gps fix position
    double gps_lastp_azimuth;   // azimuth to the previous gps fix position
    
    double gps_navp_dist;       // distance to the next navigation point
    double gps_navp_azimuth;    // azimuth to the next navigation point
    double gps_navp_maxdist;    // maximum distance to the next navigation point (needed for calibration)
    int    gps_navp_loadarea;

    int    gps_ref;
    double gps_x; 
    double gps_y;

public:
    // shared
    double ahrs_roll; 
    double ahrs_pitch; 
    double ahrs_yaw;

public:
    // shared
    double ctrlb_time1;         // when the data were received from controlboard
    int ctrlb_state;
    int ctrlb_ircv;
    double ctrlb_ircv500;
    int ctrlb_angle; 
    int ctrlb_velocity;
    int ctrlb_loadd;            // detekcia nakladu (load detection), 0 nebol detekovany sudok, 1 bol detekovany sudok

    double ctrlb_time2;         // when the data were received from controlboard
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
