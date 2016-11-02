#include "config.h"
#include "dataset.h"
  
DataSet::DataSet()
{
    image_number = -1;

    process_angle = -1;
    process_stop = -1;

    gps_fix = -1;
    gps_latitude  = ANGLE_NONE;
    gps_longitude = ANGLE_NONE;
    gps_course    = ANGLE_NONE;
    
    gps_navp_dist      = -1;
    gps_navp_azimuth   = ANGLE_NONE;
    
    ahrs_roll  = ANGLE_NONE; 
    ahrs_pitch = ANGLE_NONE; 
    ahrs_yaw   = ANGLE_NONE;

    ctrlb_euler_x = ANGLE_NONE;
    ctrlb_euler_y = ANGLE_NONE;
    ctrlb_euler_z = ANGLE_NONE;
    ctrlb_calib_gyro  = -1;
    ctrlb_calib_accel = -1;
    ctrlb_calib_mag   = -1;
}
