/***********************************************************************************************************************
<program name> -cb <control_board_device> -nogps -lidar <lidar_device> -ahrs <arhs_device> -nosave -nowait 
               -h <start_hour> -m <start_min> -cg <gps_azimuth> -ca <ahrs_yaw> -navy <yaw> -path <PxPyPz>
example:
istro_rt2016 -cb /dev/ttyUSB0 -gps /dev/ttyUSB1 -lidar /dev/ttyUSB2 -ahrs /dev/ttyACM0 -nowait
***********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "system.h"
#include "logger.h"
#include "ctrlboard.h"
#include "lidar.h"
#include "camera.h"
#include "vision.h"
#include "sample.h"
#include "gpsdev.h"
#include "geocalc.h"
#include "myahrs.h"
#include "navig.h"
#include "dataset.h"
#include "threads.h"

#ifndef WIN32
const string outputDir = "out/";
#else
const string outputDir = "out\\";
#endif

// Types of steering
#define STOPED	0
#define AUTONOMOUS	1
#define MANUAL	2

using namespace cv;
using namespace std;

LOG_DEFINE(loggerIstro, "istro");
    
Config conf;
ControlBoard ctrlBoard;
Lidar lidar;
Camera camera;
Vision vision;
GpsDevice gps;
GeoCalc geoCalc;
AHRSystem ahrs;
Threads threads;

int navigationPathPos = 0;

long save_rand = 0;

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

int loadSampleOnRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(320, 370)) < 0) return -1;
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(35, 363)) < 0) return -1;
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(172, 443)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(266, 268)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(155, 349)) < 0) return -1;
    if (sample.addSample("sample/0001179_cesta_vlavo.jpg", Point(531, 334)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(366, 359)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(250, 400)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(297, 356)) < 0) return -1;
    if (sample.addSample("sample/0000021_cesta_proti_slnku.jpg", Point(361, 340)) < 0) return -1;
    if (sample.addSample("sample/0000340_cesta.jpg", Point(262, 357)) < 0) return -1;
    if (sample.addSample("sample/0004747.jpg", Point(347, 236)) < 0) return -1;
    if (sample.addSample("sample/0004747.jpg", Point(549, 397)) < 0) return -1;
    if (sample.addSample("sample/0006832.jpg", Point(256, 211)) < 0) return -1;
    if (sample.addSample("sample/0007408.jpg", Point(280, 251)) < 0) return -1;
    if (sample.addSample("sample/0007630.jpg", Point(171, 366)) < 0) return -1;
    if (sample.addSample("sample/0008350.jpg", Point(163, 236)) < 0) return -1;
    if (sample.addSample("sample/0007775.jpg", Point(559, 373)) < 0) return -1;
    
    return 0;
}

int loadSampleOffRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(20, 200)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(235, 110)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(103, 176)) < 0) return -1;
    if (sample.addSample("sample/0001179_cesta_vlavo.jpg", Point(4, 105)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(342, 120)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(468, 73)) < 0) return -1;
//  if (sample.addSample("sample/0001227_trava_modra.jpg", Point(459, 246)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(499, 100)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(491, 70)) < 0) return -1;
    if (sample.addSample("sample/0000130_trava.jpg", Point(145, 101)) < 0) return -1;
    if (sample.addSample("sample/0000189_cesta.jpg", Point(605, 234)) < 0) return -1;
    
    return 0;
}

int initVision(void)
{
    SamplePixels sampleOnRoad;
    SamplePixels sampleOffRoad;
   
    if (loadSampleOnRoad(sampleOnRoad) < 0) {
        LOGM_ERROR(loggerIstro, "initVision", "error loading sample1!");
        return -1;
    }
    if (loadSampleOffRoad(sampleOffRoad) < 0) {
        LOGM_ERROR(loggerIstro, "initVision", "error loading sample2!");
        return -1;
    }

    vision.init(sampleOnRoad, sampleOffRoad);   
}

int initDevices(void)
{
    srand (time(NULL));
    save_rand = rand() % 1000 + (rand() % 1000) * 1000 + (1 + rand() % 9)*1000000;
    
    if (conf.useControlBoard) {
        if (conf.useControlBoard2) {
            if (ctrlBoard.init(conf.ControlBoardPortName, conf.ControlBoard2PortName) < 0) 
                return -2;
        } else {
            if (ctrlBoard.init(conf.ControlBoardPortName, NULL) < 0) 
                return -3;
        }
    }

    if (conf.useLidar) {
        if (lidar.init(conf.LidarPortName) < 0) 
            return -4;
    }

    if (conf.useCamera) {
        if (camera.init() < 0) 
            return -5;
        if (initVision() < 0)
            return -6;
    }

    if (conf.useGPSDevice) {
        if (gps.init() < 0) 
            return -7;
        if (geoCalc.init() < 0) 
            return -8;
    }

    if (conf.useAHRSystem) {
        if (ahrs.init(conf.AHRSystemPortName) < 0) 
            return -9;
    }

    
    if (threads.init(&dataset[0], &dataset[1], &dataset[2], &dataset[3], &dataset[4], &dataset[5], &dataset[6], &dataset[7], &dataset[8], &dataset[9], &dataset[10]) < 0) 
        return -10;
    
    // one dataset will be permanently set to state "THDATA_STATE_SHARED"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_SHARED);

    // another one dataset will be permanently set to state "THDATA_STATE_CTRLBOARD"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_CTRLBOARD);
    
    return 0;
} 

void closeDevices()
{
    threads.close();
    
    if (conf.useGPSDevice) {
        geoCalc.close();
        gps.close();
    }
    
    if (conf.useCamera) {
        camera.close();
    }    
 
    if (conf.useLidar) {
        lidar.close();
    }
    
    if (conf.useControlBoard) {
       ctrlBoard.close();
    }    
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstro, "waitForStart", "start sleep...");

	    sleep(conf.waitDelay);
        LOGM_INFO(loggerIstro, "waitForStart", "wake up!");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstro, "waitForStart", "start time sleep...");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstro, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstro, "waitForStart", "wake up!");
    }    
}

void saveImage(long image_number, const string& str, const Mat& img)
{
    char filename[256];
//  long tick_number = getTickCount();

    if (!img.empty()) {
        sprintf(filename, "%srobotour_%07u_%07u_%s.jpg", outputDir.c_str(), save_rand, image_number, str.c_str());
        imwrite(filename, img);
    }    
}

int gps_writeData(int gps_fix, double gps_latitude, double gps_longitude, double gps_course, double gps_lastp_dist, double gps_lastp_azimuth, double gps_navp_dist, double gps_navp_azimuth) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_fix = gps_fix;
    pdata->gps_latitude = gps_latitude;
    pdata->gps_longitude = gps_longitude;
    pdata->gps_course = gps_course;

    pdata->gps_lastp_dist = gps_lastp_dist;
    pdata->gps_lastp_azimuth = gps_lastp_azimuth;
    
    pdata->gps_navp_dist = gps_navp_dist;
    pdata->gps_navp_azimuth = gps_navp_azimuth;
    
    LOGM_DEBUG(loggerIstro, "gps_writeData", "fix=" << gps_fix << ", latitude=" << ioff(gps_latitude, 6) << ", longitude=" << ioff(gps_longitude, 6) << ", course=" << ioff(gps_course, 2) << 
        ", lastp_dist=" << ioff(gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(gps_lastp_azimuth, 2) << ", navp_dist=" << ioff(gps_navp_dist, 3) << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2));
    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::gps_writeData", t);
}

void *gps_thread(void *parg)
{
    int gps_fix;
    double gps_latitude;
    double gps_longitude;
    double gps_course;

    // previous gps fix position
    double lastp_latitude = ANGLE_NONE;
    double lastp_longitude = ANGLE_NONE;
    double gps_lastp_dist = -1;
    double gps_lastp_azimuth = ANGLE_NONE;

    // next navigation point    
    double navp_latitude = ANGLE_NONE;
    double navp_longitude = ANGLE_NONE;
    double gps_navp_dist = -1;
    double gps_navp_azimuth = ANGLE_NONE;
    
    double t;
    int result = 0;
    
    int point = 0;
    
    LOG_THREAD_NAME("gps");
    LOGM_INFO(loggerIstro, "gps_thread", "start");

    if (conf.useNavigation) {
        navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude, navp_longitude);
    }
    
    while ((!thread_testcancel()) && conf.useGPSDevice) {
        t = timeBegin();

        if ((gps_fix = gps.getData(gps_latitude, gps_longitude, gps_course)) < 0) {
            result = -2;
            break;
        }
        if (gps_fix > 0) {
            if ((lastp_latitude < ANGLE_OK) && (lastp_latitude < ANGLE_OK)) {         
                geoCalc.getDist(lastp_latitude, lastp_longitude, gps_latitude, gps_longitude, gps_lastp_dist, gps_lastp_azimuth);
            } else {
                gps_lastp_dist = -1;
                gps_lastp_azimuth = ANGLE_NONE;
            }
            
            if (conf.useNavigation && (navp_latitude < ANGLE_OK) && (navp_latitude < ANGLE_OK)) {         
                geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude, navp_longitude, gps_navp_dist, gps_navp_azimuth);
                //printf("gps_thread: geoCalc.getDist(): pos=%d, dist=%f, azimuth=%f\n", navigationPathPos, gps_navp_dist, gps_navp_azimuth);
                if (gps_navp_dist < NAVIGATION_DISTANCE_THRESHOLD) {
                    LOGM_INFO(loggerIstro, "gps_thread", "pos=" << navigationPathPos << " passed!");
                    navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude, navp_longitude);
                }
            } else {
                gps_navp_dist = -1;
                gps_navp_azimuth = ANGLE_NONE;
            }
            
            lastp_latitude = gps_latitude;
            lastp_longitude = gps_longitude;
        } else {
            gps_navp_dist = -1;
            gps_navp_azimuth = ANGLE_NONE;
            LOGM_WARN(loggerIstro, "gps_thread", "no gps-fix!");
        }

        timeEnd("istro::gps_thread.capture", t);

        if (gps_writeData(gps_fix, gps_latitude, gps_longitude, gps_course, gps_lastp_dist, gps_lastp_azimuth, gps_navp_dist, gps_navp_azimuth) < 0) {
            result = -1;
            break;
        }
        
        LOGM_DEBUG(loggerIstro, "gps_thread", "data updated");
    }

    LOGM_INFO(loggerIstro, "gps_thread", "exit(" << result << ")");
    thread_exit();
}

int ahrs_writeData(int ahrs_roll, double ahrs_pitch, double ahrs_yaw) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ahrs_roll = ahrs_roll; 
    pdata->ahrs_pitch = ahrs_pitch; 
    pdata->ahrs_yaw = ahrs_yaw;
    
    LOGM_DEBUG(loggerIstro, "ahrs_writeData", "ahrs_roll=" << ioff(ahrs_roll, 2) << ", ahrs_pitch=" << ioff(ahrs_pitch, 2) << ", ahrs_yaw=" << ioff(ahrs_yaw, 2));
    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::ahrs_writeData", t);
}

void *ahrs_thread(void *parg)
{
    double ahrs_roll; 
    double ahrs_pitch; 
    double ahrs_yaw;

    double t;
    int result = 0;

    LOG_THREAD_NAME("ahrs");
    LOGM_INFO(loggerIstro, "ahrs_thread", "start");

    int cnt = 0;
    while ((!thread_testcancel()) && conf.useAHRSystem) {
        t = timeBegin();

        if (ahrs.getData(ahrs_roll, ahrs_pitch, ahrs_yaw) < 0) {
            result = -1;
            break;
        }

        timeEnd("istro::ahrs_thread.capture", t);

        // write every fifth value (50ms)
        if (cnt++ > 5) {
            cnt = 0;
            if (ahrs_writeData(ahrs_roll, ahrs_pitch, ahrs_yaw) < 0) {
                result = -2;
                break;
            }
            LOGM_DEBUG(loggerIstro, "ahrs_thread", "data updated");
        }
    }

    LOGM_INFO(loggerIstro, "ahrs_thread", "exit(" << result << ")");
    thread_exit();
}

int ctrlBoard_writeData(double &ctrlb_euler_x, double &ctrlb_euler_y, double &ctrlb_euler_z, 
        int &ctrlb_calib_gyro, int &ctrlb_calib_accel, int &ctrlb_calib_mag) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ctrlb_euler_x = ctrlb_euler_x;
    pdata->ctrlb_euler_y = ctrlb_euler_y;
    pdata->ctrlb_euler_z = ctrlb_euler_z;
    pdata->ctrlb_calib_gyro = ctrlb_calib_gyro; 
    pdata->ctrlb_calib_accel = ctrlb_calib_accel;
    pdata->ctrlb_calib_mag = ctrlb_calib_mag;  
    
    LOGM_DEBUG(loggerIstro, "ctrlBoard_writeData", "euler_x=" << ioff(ctrlb_euler_x, 2) << ", euler_y=" << ioff(ctrlb_euler_y, 2) << ", euler_z=" << ioff(ctrlb_euler_z, 2) << 
        ", calib_gyro=" << ctrlb_calib_gyro << ", calib_accel=" << ctrlb_calib_accel << ", calib_mag=" << ctrlb_calib_mag);
            
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::ctrlBoard_writeData", t);
}

void *ctrlBoard_thread(void *parg)
{
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int    ctrlb_calib_gyro;
    int    ctrlb_calib_accel;
    int    ctrlb_calib_mag;

    double t;
    int res;
    int result = 0;
    
    LOG_THREAD_NAME("ctrlBoard");
    LOGM_INFO(loggerIstro, "ctrlBoard_thread", "start");

    int cnt = 0;
    while ((!thread_testcancel()) && (conf.useControlBoard || conf.useControlBoard2)) {
        t = timeBegin();

        if ((res = ctrlBoard.getImuData(ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag)) < 0) {
            result = -1;
            break;
        }

        timeEnd("istro::ctrlBoard_thread.capture", t);
        if (res > 0) {
            if (ctrlBoard_writeData(ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag) < 0) {
                result = -2;
                break;
            }
            LOGM_DEBUG(loggerIstro, "ctrlBoard_thread", "data updated");
        } else {
            // sleep because we are performing asynchronous reads (expecting data 10 times per second)
            msleep(50);
        }
    }

    LOGM_INFO(loggerIstro, "ctrlBoard_thread", "exit(" << result << ")");
    thread_exit();
}

void *capture_camera_thread(void *parg)
{
    double t;
    int result = 0;
    
    DataSet *pdata;

    LOG_THREAD_NAME("capture_camera");
    LOGM_INFO(loggerIstro, "capture_camera_thread", "start");

    while ((!thread_testcancel()) && conf.useCamera) {
        t = timeBegin();
        pdata = (DataSet *)threads.getData(THDATA_STATE_NEW, THDATA_STATE_CAMERA_CAPTURING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::capture_camera_thread.wait", t);

        t = timeBegin();
        if (conf.useCamera) {
            if (camera.getFrame(pdata->camera_img) < 0) {
                threads.setData(pdata, THDATA_STATE_NEW, -1);
                result = -2;
                break;
            }
        }
        timeEnd("istro::capture_camera_thread.capture", t);        

        threads.setData(pdata, THDATA_STATE_CAMERA_CAPTURED, 1);
        LOGM_DEBUG(loggerIstro, "capture_camera_thread", "data captured");
    }

    LOGM_INFO(loggerIstro, "capture_camera_thread", "exit(" << result << ")");
    thread_exit();
}

void *capture_lidar_thread(void *parg)
{
    double t;
    int result = 0;
    
    DataSet *pdata;

    LOG_THREAD_NAME("capture_lidar");
    LOGM_INFO(loggerIstro, "capture_lidar_thread", "start");

    while ((!thread_testcancel()) && conf.useLidar) {
        t = timeBegin();        
        pdata = (DataSet *)threads.getData(THDATA_STATE_NEW, THDATA_STATE_LIDAR_CAPTURING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::capture_lidar_thread.wait", t);

        t = timeBegin();
        if (conf.useLidar) {
            if (lidar.getData(pdata->lidar_data, pdata->lidar_data_cnt) < 0) {
                threads.setData(pdata, THDATA_STATE_NEW, -1);
                result = -3;
                break;
            }
        }
        timeEnd("istro::capture_lidar_thread.capture", t);

        threads.setData(pdata, THDATA_STATE_LIDAR_CAPTURED, 1);
        LOGM_DEBUG(loggerIstro, "capture_lidar_thread", "data captured");
    }

    LOGM_INFO(loggerIstro, "capture_lidar_thread", "exit(" << result << ")");
    thread_exit();
}

const int DMAP_MIN_INTERVAL_LENGTH = 20;

int process_writeData(long image_number, int process_angle, int process_stop) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->image_number = image_number;
    pdata->process_angle = process_angle;
    pdata->process_stop = process_stop;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::process_writeData", t);
}

int process_readData(double &gps_navp_azimuth, double &ctrlb_euler_x, double &ahrs_yaw, double &gps_lastp_azimuth) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    gps_navp_azimuth = pdata->gps_navp_azimuth;
    ahrs_yaw = pdata->ahrs_yaw;
    ctrlb_euler_x = pdata->ctrlb_euler_x;
    gps_lastp_azimuth = pdata->gps_lastp_azimuth;
    
    LOGM_DEBUG(loggerIstro, "process_readData", "fix=" << pdata->gps_fix << ", latitude=" << ioff(pdata->gps_latitude, 6) << ", longitude=" << ioff(pdata->gps_longitude, 6) << ", course=" << ioff(pdata->gps_course, 2) << 
        ", lastp_dist=" << ioff(pdata->gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(gps_lastp_azimuth, 2) << 
        ", euler_x=" << ioff(pdata->ctrlb_euler_x, 2) << ", ahrs_yaw=" << ioff(pdata->ahrs_yaw, 2) << 
        ", navp_dist=" << ioff(pdata->gps_navp_dist, 3) << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2));
    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::process_readData", t);
}

void *process_thread(void *parg)
{
    double t;
    int result = 0;
    long image_number = 0;
    
    DataSet *pdata;
    DataSet *pdata2;

    LOG_THREAD_NAME("process");
    LOGM_INFO(loggerIstro, "process_thread", "start");

    while (!thread_testcancel() && (conf.useCamera || conf.useLidar)) {
        t = timeBegin();
        void *ptr = NULL;
        if ((conf.useCamera && conf.useLidar)) {
            pdata = (DataSet *)threads.getData2(THDATA_STATE_CAMERA_CAPTURED, THDATA_STATE_PROCESSING, &ptr, THDATA_STATE_LIDAR_CAPTURED);
        } else {
            if (conf.useCamera) {
                pdata = (DataSet *)threads.getData(THDATA_STATE_CAMERA_CAPTURED, THDATA_STATE_PROCESSING);
            } 
            if (conf.useLidar) {
                pdata = (DataSet *)threads.getData(THDATA_STATE_LIDAR_CAPTURED, THDATA_STATE_PROCESSING);                
            }
        }
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::process_thread.wait", t);

        t = timeBegin();
        if (ptr != NULL) {
            // merge lidar data to camera dataset (fixme: DataSet:copyTo)        
            pdata2 = (DataSet *)ptr;
            pdata->lidar_data_cnt = pdata2->lidar_data_cnt;
            memcpy(pdata->lidar_data, pdata2->lidar_data, LIDAR_DATA_NUM * sizeof(lidar_data_t));
            threads.setData(pdata2, THDATA_STATE_NEW, -1);
        }
        
        pdata->process_dir = 90;
        pdata->process_stop = 0;
        // set dmap to "1" - no obstacles around
        dmap_set(pdata->process_dmap, 1);

        if (conf.useCamera) {
            vision.eval(pdata->camera_img, pdata->vision_markers, pdata->vision_markersIM, pdata->vision_epweigth, pdata->vision_elweigth, pdata->vision_dmap);
            dmap_apply(pdata->process_dmap, pdata->vision_dmap);
            //dmap_print(pdata->vision_dmap, "process_thread(): vision");
            
            dmap_find(pdata->vision_dmap, DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->vision_angle_min, pdata->vision_angle_max);
            LOGM_INFO(loggerIstro, "process_thread", "dmap_camera: vision_angle_min=" << pdata->vision_angle_min << ", vision_angle_max=" << pdata->vision_angle_max);
        }

        if (conf.useLidar) {
            lidar.process(pdata->lidar_data, pdata->lidar_data_cnt, pdata->lidar_dmap, pdata->lidar_stop);
            dmap_apply(pdata->process_dmap, pdata->lidar_dmap);
            //dmap_print(pdata->lidar_dmap, "process_thread(): lidar");
            
            if (pdata->lidar_stop > 0) {
                pdata->process_stop = 1;
            }
            
            dmap_find(pdata->lidar_dmap, DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->lidar_angle_min, pdata->lidar_angle_max);
            LOGM_INFO(loggerIstro, "process_thread", "dmap_lidar: lidar_angle_min=" << pdata->lidar_angle_min << ", lidar_angle_max=" << pdata->lidar_angle_max << ", lidar_stop=" << pdata->lidar_stop);
        }
        
        if (process_readData(pdata->gps_navp_azimuth, pdata->ctrlb_euler_x, pdata->ahrs_yaw, pdata->gps_lastp_azimuth) < 0) {
            result = -2;
            break;
        }

        int yaw_src = 0;
        double yaw = pdata->ctrlb_euler_x;
        if (yaw >= ANGLE_OK) {
            yaw = pdata->ahrs_yaw;
            yaw_src = 1;
        }
        if (yaw >= ANGLE_OK) {
            yaw = pdata->gps_lastp_azimuth;
            yaw_src = 2;
        }        
        
        dmap_find(pdata->process_dmap, DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->process_angle_min, pdata->process_angle_max);
#ifdef WIN32
//LOGM_WARN(loggerIstro, "process_thread", "clear process_angle_min/max, DEBUG ONLY!!");
//pdata->process_angle_min = 0;
//pdata->process_angle_max = 180;
#endif

        pdata->process_angle = -1;
        if ((pdata->process_angle_min == 0) && (pdata->process_angle_max == 180)) {
            if ((conf.navigationImuYaw < (int)ANGLE_OK) && (yaw < ANGLE_OK)) {
                double angle = conf.navigationImuYaw - yaw;
                while (angle > 180) {
                    angle -= 360; 
                }
                while (angle <= -180) {
                    angle += 360;
                }
                if ((angle > -180) && (angle <= 180)) {
                    pdata->process_angle = 90 - trunc(angle);
                    if (pdata->process_angle < NAVIGATION_ANGLE_MIN) {
                        pdata->process_angle = NAVIGATION_ANGLE_MIN;
                    }
                    if (pdata->process_angle > NAVIGATION_ANGLE_MAX) {
                        pdata->process_angle = NAVIGATION_ANGLE_MAX;
                    }
                }
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"YAW\"): nav_yaw=" << conf.navigationImuYaw << ", yaw=" << ioff(yaw, 2) << ", process_angle=" << pdata->process_angle << ", yaw_src=" << yaw_src);
            } else
            if ((pdata->gps_navp_azimuth < ANGLE_OK) && (yaw < ANGLE_OK)) {
                double angle = pdata->gps_navp_azimuth - conf.calibGpsAzimuth + conf.calibImuYaw - yaw;
                while (angle > 180) {
                    angle -= 360; 
                }
                while (angle <= -180) {
                    angle += 360;
                }
                if ((angle > -180) && (angle <= 180)) {
                    pdata->process_angle = 90 - trunc(angle);
                    if (pdata->process_angle < NAVIGATION_ANGLE_MIN) {
                        pdata->process_angle = NAVIGATION_ANGLE_MIN;
                    }
                    if (pdata->process_angle > NAVIGATION_ANGLE_MAX) {
                        pdata->process_angle = NAVIGATION_ANGLE_MAX;
                    }
                }
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"AZIMUTH_YAW\"): navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2) << ", process_angle=" << pdata->process_angle << ", yaw_src=" << yaw_src);
            }
        }
        if ((pdata->process_angle_min >= 0) && (pdata->process_angle_max >= 0)) {
            if (pdata->process_angle == -1) {
                pdata->process_angle = (pdata->process_angle_min + pdata->process_angle_max)/2; 
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"MIN_MAX\"): navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", yaw=" << ioff(yaw, 2) << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max << ", process_angle=" << pdata->process_angle << ", yaw_src=" << yaw_src);
            }
        }

        pdata->image_number = image_number++;
        timeEnd("istro::process_thread.process", t);
        
        if (process_writeData(pdata->image_number, pdata->process_angle, pdata->process_stop) < 0) {
            result = -3;
            break;
        }
        
        threads.setData(pdata, THDATA_STATE_PROCESSED, 2);

        LOGM_INFO(loggerIstro, "process_thread", "data processed, image=" << image_number-1);
    }

    LOGM_INFO(loggerIstro, "process_thread", "exit(" << result << ")");
    thread_exit();
}

void *save_thread(void *parg)
{
    double t;
    int result = 0;
    Mat vision_img;
    Mat lidar_img;
    
    DataSet *pdata;

    LOG_THREAD_NAME("save");
    LOGM_INFO(loggerIstro, "save_thread", "start");

    while (!thread_testcancel()) {
        t = timeBegin();
        pdata = (DataSet *)threads.getData(THDATA_STATE_PROCESSED, THDATA_STATE_SAVING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::save_thread.wait", t);

        t = timeBegin();
        if (!conf.useNosave) { 
            if (conf.useCamera) {
                camera.drawFrame(pdata->camera_img);
                saveImage(pdata->image_number, "camera", pdata->camera_img);
            
                vision.drawOutput(pdata->camera_img, vision_img, pdata->vision_markers, pdata->vision_epweigth, pdata->vision_elweigth, pdata->vision_angle_min, pdata->vision_angle_max);
                saveImage(pdata->image_number, "vision", vision_img);
            }
        
            if (conf.useLidar) {
                lidar.drawOutput(pdata->lidar_data, pdata->lidar_data_cnt, lidar_img, pdata->lidar_dmap, pdata->lidar_stop, pdata->lidar_angle_min, pdata->lidar_angle_max, pdata->process_angle, pdata->process_angle_min, pdata->process_angle_max, pdata->image_number);
                saveImage(pdata->image_number, "lidar", lidar_img);
            }
        }
        timeEnd("istro::save_thread.save", t);
        
        threads.setData(pdata, THDATA_STATE_NEW, -1);    //threads.cleanData(THDATA_STATE_SAVED, THDATA_STATE_NEW);
        LOGM_DEBUG(loggerIstro, "save_thread", "data saved");
    }

    LOGM_INFO(loggerIstro, "save_thread", "exit(" << result << ")");
    thread_exit();
}

int loop_readData(int &process_angle, int &process_stop) 
{
    DataSet *pdata;
    
    double t = timeBegin();
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    process_angle = pdata->process_angle;
    process_stop = pdata->process_stop;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    timeEnd("istro::loop_readData", t);
}

int loop(void)
{
    int steering_change = 0;
    int steering_type = AUTONOMOUS;
    int steering_angle = SA_STRAIGHT;
    
    int velocity_change = 1;
    int velocity = VEL_OPT;    // initial speed
    int velocity_stop = 0;
    
    if (conf.useControlBoard) {
       sleep(3);
       ctrlBoard.start();
    }

    LOGM_INFO(loggerIstro, "loop", "start");
    
#ifdef ISTRO_GUI
    if ((!conf.useCamera) && (!conf.useLidar)) 
#endif        
    {
        // fixme: empty window for waitKey() - waitKey() does not work without any other windows
        Mat img_empty(100, 100, CV_8UC3, Scalar( 0,0,0) );
        imshow("empty", img_empty);
    }
    
    if (threads.start(capture_camera_thread, capture_lidar_thread, gps_thread, ahrs_thread, ctrlBoard_thread, process_thread, save_thread) < 0) {
        return -1;
    }
    
    for(;;) {
        steering_change = 0;
        
        if (steering_type == AUTONOMOUS) {
            int process_stop = -1;
            int process_angle = -1;

            if (loop_readData(process_angle, process_stop) < 0) {
                break;
            }
                        
            if (process_angle >= 0) {
                steering_angle = process_angle + SA_FIX;
                if (conf.useControlBoard) {
                    ctrlBoard.setSteeringAngle(steering_angle);
                }
            }
            
            if (process_stop >= 0) {
                if ((process_stop > 0) && (velocity_stop == 0)) {
                    velocity_stop = 1;
                    if (conf.useControlBoard) {
                        ctrlBoard.setSpeed(VEL_ZERO);
                    }
                }
                if ((process_stop == 0) && (velocity_stop > 0)) {
                    velocity_stop = 0;
                    if (conf.useControlBoard) {
                        ctrlBoard.setSpeed(velocity);
                    }
                }
            }
        }
           
        char c = (char) waitKey(20);
        if( c == 27 )
            break;
        
        switch(c) {
        case 'q':
        case 'Q':
            //left
            steering_type = MANUAL;
            steering_angle = SA_MAX;
            LOGM_INFO(loggerIstro, "loop", "steering_angle=" << steering_angle);
            steering_change = 1;
            break;
        case 'e':
        case 'E':
            //right
            steering_type = MANUAL;
            steering_angle=SA_MIN;
            LOGM_INFO(loggerIstro, "loop", "steering_angle=" << steering_angle);
            steering_change = 1;
            break;
        case 'a':
        case 'A':
            //left
            steering_type = MANUAL;
            steering_angle+=10;
            LOGM_INFO(loggerIstro, "loop", "steering_angle=" << steering_angle);
            steering_change = 1;
            break;
        case 'd':
        case 'D':
            //right
            steering_type = MANUAL;
            steering_angle-=10;
            LOGM_INFO(loggerIstro, "loop", "steering_angle=" << steering_angle);
            steering_change = 1;
            break;
/*      case 'w':
        case 'W':
            //forward
            printf("Vel++\n");
            break;
        case 's':
        case 'S':
            //forward
            printf("Vel--\n");
            break;  */
        case ' ':
            //Stop
            steering_type = MANUAL;
            steering_angle = SA_STRAIGHT;
            steering_change = 1;
            velocity = VEL_ZERO;
            velocity_change = 1;
            LOGM_INFO(loggerIstro, "loop", "stop!");
            break;
        case 'm':
        case 'M':
            //Manual steering
            steering_type = MANUAL;
            LOGM_INFO(loggerIstro, "loop", "steering_type=\"MANUAL\"");
            break;
        case 'u':
        case 'U':
            //Autonomous steering
            steering_type = AUTONOMOUS;
            LOGM_INFO(loggerIstro, "loop", "steering_type=\"AUTONOMOUS\"");
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            velocity = VEL_ZERO + c - '0';
            if (velocity_stop > 0) {
                velocity_stop = 0;
                steering_type = MANUAL;
                LOGM_INFO(loggerIstro, "loop", "steering_type=\"MANUAL\"");
            }
            LOGM_INFO(loggerIstro, "loop", "velocity=" << velocity);
            velocity_change = 1;
            break;
        default:
            ;
        }

        if (steering_change) {
            steering_change = 0;            
            if (conf.useControlBoard) {
                ctrlBoard.setSteeringAngle(steering_angle);
            }
        }
        if (velocity_change) {
            velocity_change = 0;            
            if (conf.useControlBoard) {
                ctrlBoard.setSpeed(velocity);
            }
        }
    }
    
    threads.stop();
}

int main(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstro, "-----------------------------");
    LOGM_INFO(loggerIstro, "main", "application start");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstro, "main", "parseArguments failed!");
        return -1;
    }
    conf.printArguments();

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstro, "main", "initDevices failed!");
        return -1;
    }
    
    waitForStart();

    loop();
 
    closeDevices();
    
    thread_exit();

    LOGM_INFO(loggerIstro, "main", "application exit");
    return 0;
}
