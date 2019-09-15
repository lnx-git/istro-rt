#include "config.h"
#include "navig.h"
#include "dataset.h"
  
DataSet::DataSet()
{
    image_number = 0;

    qrscan_latitude  = ANGLE_NONE;
    qrscan_longitude = ANGLE_NONE;

    process_angle = (int)ANGLE_NONE;
    process_velocity = -1;
    process_stop = -1;
    process_state = PROCESS_STATE_NONE;

    process_yaw = ANGLE_NONE;
    process_ref = 0;
    process_x   = 0.0;
    process_y   = 0.0;
    process_time = -1;

    process_dir = 90;
    process_angle_min = -1;
    process_angle_max = -1;

    coneseek_angle_min = -1;
    coneseek_angle_max = -1;
    coneseek_intlen = -1;
    coneseek_stop = -1;

    gps_time = -1;

    gps_fix = -1;
    gps_latitude  = ANGLE_NONE;
    gps_longitude = ANGLE_NONE;
    gps_latitude_raw  = ANGLE_NONE;
    gps_longitude_raw = ANGLE_NONE;
    gps_speed     = -1;
    gps_course    = ANGLE_NONE;

    gps_lastp_dist = -1;
    gps_lastp_azimuth = ANGLE_NONE;

    gps_navp_idx         = -1;
    gps_navp_dist        = -1;
    gps_navp_dist_raw    = -1;
    gps_navp_azimuth     = ANGLE_NONE;
    gps_navp_azimuth_raw = ANGLE_NONE;
    gps_navp_maxdist     = -1;
    gps_navp_loadarea    = NAVIGATION_AREA_NONE;
    gps_navp_latitude    = ANGLE_NONE;
    gps_navp_longitude   = ANGLE_NONE;
    gps_navp_latitude_raw  = ANGLE_NONE;
    gps_navp_longitude_raw = ANGLE_NONE;

    gps_ref = 0;
    gps_x   = 0.0; 
    gps_y   = 0.0;
    
    ahrs_roll  = ANGLE_NONE; 
    ahrs_pitch = ANGLE_NONE; 
    ahrs_yaw   = ANGLE_NONE;

    ctrlb_time1       = -1;
    ctrlb_state       = -1;
    ctrlb_ircv        = -1;
    ctrlb_ircv500     = -1;
    ctrlb_angle       = -1; 
    ctrlb_velocity    = -1;
    ctrlb_loadd       = -1;

    ctrlb_time2       = -1;
    ctrlb_euler_x     = ANGLE_NONE;
    ctrlb_euler_y     = ANGLE_NONE;
    ctrlb_euler_z     = ANGLE_NONE;
    ctrlb_calib_gyro  = -1;
    ctrlb_calib_accel = -1;
    ctrlb_calib_mag   = -1;
}
