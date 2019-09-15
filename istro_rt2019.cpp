/***********************************************************************************************************************
<program name> -cb <control_board_device> -nogps -lidar <lidar_device> -ahrs <arhs_device> -nosave -nowait 
               -h <start_hour> -m <start_min> -cg <gps_azimuth> -ca <ahrs_yaw> -navy <yaw> -path <PxPyPz>
example:
istro_rt2019 -cb /dev/ttyUSB0 -gps /dev/ttyUSB1 -lidar /dev/ttyUSB2 -ahrs /dev/ttyACM0 -nowait
***********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <exception>
#include <string.h>
#include "threads.h"
#include "config.h"
#include "system.h"
#include "logger.h"
#include "ctrlboard.h"
#include "lidar.h"
#include "camera.h"
#include "vision.h"
#include "mtime.h"
#include "sample.h"
#include "gpsdev.h"
#include "geocalc.h"
#include "myahrs.h"
#include "navig.h"
#include "wmodel.h"
#include "qrscan.h"
#include "navmap.h"
#include "dataset.h"

#ifndef WIN32
const string outputDir = "out/";
#else
const string outputDir = "out\\";
#endif

// Types of steering
#define STOPED        0
#define AUTONOMOUS    1
#define MANUAL        2

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
WorldModel wmodel;
QRScanner qrscan;
Threads threads;

int navigationPathPos = 0;

long save_rand = 0;

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

const int GPS_POINT_NUM = 30;

// fixme: global variables, not thread safe - too "big" to store in dataset
int         gps_pt_cnt = 0;
aux_point_t gps_pt[GPS_POINT_NUM + 1];

#ifdef ISTRO_VISION_ORANGECONE

int loadSampleOnRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(313, 341)) < 0) return -1;
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(249, 328)) < 0) return -1;
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(378, 323)) < 0) return -1;
    if (sample.addSample("sample/2009591_kuzele.jpg", Point(214, 78)) < 0) return -1;
    if (sample.addSample("sample/2009591_kuzele.jpg", Point(330, 213)) < 0) return -1;
    if (sample.addSample("sample/2007147_kuzele.jpg", Point(409, 220)) < 0) return -1;
    if (sample.addSample("sample/2007147_kuzele.jpg", Point(470, 190)) < 0) return -1;
    if (sample.addSample("sample/2010533_kuzele.jpg", Point(428, 217)) < 0) return -1;
    if (sample.addSample("sample/2021474_kuzele.jpg", Point(142, 354)) < 0) return -1;
    if (sample.addSample("sample/2010509_kuzele.jpg", Point(455, 219)) < 0) return -1;
    if (sample.addSample("sample/2013222_kuzele.jpg", Point(314, 163)) < 0) return -1;
    if (sample.addSample("sample/2008241_kuzele.jpg", Point(566, 303)) < 0) return -1;
    return 0;
}

int loadSampleOffRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/2004792_kuzele.jpg", Point(472, 403)) < 0) return -1;
    if (sample.addSample("sample/2004731_kuzele.jpg", Point(388, 245)) < 0) return -1;
    if (sample.addSample("sample/2013376_kuzele2.jpg", Point(201, 137)) < 0) return -1;
    if (sample.addSample("sample/2002698_kuzele.jpg", Point(284, 80)) < 0) return -1;
    if (sample.addSample("sample/2006395_kuzele.jpg", Point(320, 87)) < 0) return -1;
    if (sample.addSample("sample/2014689_kuzele.jpg", Point(148, 267)) < 0) return -1;
    return 0;
}

#else

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
    if (sample.addSample("sample/0104747.jpg", Point(347, 236)) < 0) return -1;
    if (sample.addSample("sample/0104747.jpg", Point(549, 397)) < 0) return -1;
    if (sample.addSample("sample/0106832.jpg", Point(256, 211)) < 0) return -1;
    if (sample.addSample("sample/0107408.jpg", Point(280, 251)) < 0) return -1;
    if (sample.addSample("sample/0107630.jpg", Point(171, 366)) < 0) return -1;
    if (sample.addSample("sample/0108350.jpg", Point(163, 236)) < 0) return -1;
    if (sample.addSample("sample/0107775.jpg", Point(559, 373)) < 0) return -1;

    if (sample.addSample("sample/0201759_rr2017.jpg", Point(376, 393)) < 0) return -1;
//  if (sample.addSample("sample/0200409_rr2017.jpg", Point(365, 407)) < 0) return -1;
    if (sample.addSample("sample/0200412_rr2017.jpg", Point(309, 404)) < 0) return -1;
    if (sample.addSample("sample/0200418_rr2017.jpg", Point(351, 361)) < 0) return -1;
    if (sample.addSample("sample/0200086_rr2017.jpg", Point(536, 438)) < 0) return -1;

    if (sample.addSample("sample/0320078_rt2018.jpg", Point(25, 442)) < 0) return -1;
    if (sample.addSample("sample/0405484_rr2019.jpg", Point(238, 109)) < 0) return -1;
    if (sample.addSample("sample/0405484_rr2019.jpg", Point(203, 93)) < 0) return -1;
    if (sample.addSample("sample/0400864_rr2019.jpg", Point(21, 401)) < 0) return -1;

    return 0;
}

int loadSampleOffRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(20, 200)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(235, 110)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(103, 176)) < 0) return -1;
    if (sample.addSample("sample/0001179_cesta_vlavo.jpg", Point(4, 105)) < 0) return -1;

/*
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(342, 120)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(468, 73)) < 0) return -1;
//  if (sample.addSample("sample/0001227_trava_modra.jpg", Point(459, 246)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(499, 100)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(491, 70)) < 0) return -1;
    if (sample.addSample("sample/0000130_trava.jpg", Point(145, 101)) < 0) return -1;
    if (sample.addSample("sample/0000189_cesta.jpg", Point(605, 234)) < 0) return -1;

    if (sample.addSample("sample/0409562_rr2019.jpg", Point(454, 207)) < 0) return -1;
    if (sample.addSample("sample/0411049_rr2019.jpg", Point(539, 147)) < 0) return -1;
*/

#ifdef ISTRO_VISION_YELLOWLANE
    if (sample.addSample("sample/1013001_zlta.jpg", Point(77, 348)) < 0) return -1;
    if (sample.addSample("sample/1013133_zlta.jpg", Point(145, 428)) < 0) return -1;
    if (sample.addSample("sample/1013324_zlta.jpg", Point(390, 356)) < 0) return -1;
    if (sample.addSample("sample/1014326_zlta.jpg", Point(191, 326)) < 0) return -1;
    if (sample.addSample("sample/1014779_zlta.jpg", Point(208, 306)) < 0) return -1;
    if (sample.addSample("sample/1014888_zlta.jpg", Point(235, 420)) < 0) return -1;
    if (sample.addSample("sample/1004748_zlta.jpg", Point(14, 358)) < 0) return -1;    
#endif
    
    return 0;
}

#endif

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
    return 0;
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
        // fixme
        gps_pt[0].style[0] = 0;
        if (gps.init() < 0) 
            return -7;
        if (geoCalc.init() < 0) 
            return -8;
    }

    if (conf.useAHRSystem) {
        if (ahrs.init(conf.AHRSystemPortName) < 0) 
            return -9;
    }

    if (conf.useQRScan) {
        qrscan.init();
    }
    
    if (navigation_init(&geoCalc) < 0)
        return -10;
    
    if (navmap_init() < 0)
        return -11;

    if (threads.init(&dataset[0], &dataset[1], &dataset[2], &dataset[3], &dataset[4], &dataset[5], &dataset[6], 
            &dataset[7], &dataset[8], &dataset[9], &dataset[10], &dataset[11], &dataset[12]) < 0) 
        return -12;
    
    // one dataset will be permanently set to state "THDATA_STATE_SHARED"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_SHARED);

    // another one dataset will be permanently set to state "THDATA_STATE_CTRLBOARD"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_CTRLBOARD);
    
    return 0;
} 

void closeDevices()
{
    LOGM_INFO(loggerIstro, "closeDevices", "msg=\"start\"");

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

    LOGM_INFO(loggerIstro, "closeDevices", "msg=\"finish\"");
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstro, "waitForStart", "msg=\"start sleep...\"");

        sleep(conf.waitDelay);
        LOGM_INFO(loggerIstro, "waitForStart", "msg=\"wake up!\"");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstro, "waitForStart", "msg=\"start time sleep...\"");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstro, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstro, "waitForStart", "msg=\"wake up!\"");
    }    
}
              
int gps_writeData(double gps_time, int gps_fix, double gps_latitude, double gps_longitude, double gps_latitude_raw, double gps_longitude_raw, 
        double gps_speed, double gps_course, 
        double gps_lastp_dist, double gps_lastp_azimuth, 
        double gps_navp_dist, double gps_navp_dist_raw, double gps_navp_azimuth, double gps_navp_azimuth_raw,
        double gps_navp_maxdist, double gps_navp_mindist, 
        double gps_navp_latitude, double gps_navp_longitude, double gps_navp_latitude_raw, double gps_navp_longitude_raw, int gps_navp_loadarea, int gps_navp_idx,
        int gps_ref, double gps_x, double gps_y, double gps_ref_latitude, double gps_ref_longitude, int nmap_flags, int coneseek_stop) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_time = gps_time;
    pdata->gps_fix = gps_fix;
    pdata->gps_latitude = gps_latitude;
    pdata->gps_longitude = gps_longitude;
    pdata->gps_latitude_raw = gps_latitude_raw;
    pdata->gps_longitude_raw = gps_longitude_raw;
    pdata->gps_speed = gps_speed;
    pdata->gps_course = gps_course;

    pdata->gps_lastp_dist = gps_lastp_dist;
    pdata->gps_lastp_azimuth = gps_lastp_azimuth;
    
    pdata->gps_navp_idx = gps_navp_idx;
    pdata->gps_navp_dist = gps_navp_dist;
    pdata->gps_navp_dist_raw = gps_navp_dist_raw;
    pdata->gps_navp_azimuth = gps_navp_azimuth;
    pdata->gps_navp_azimuth_raw = gps_navp_azimuth_raw;
    pdata->gps_navp_maxdist = gps_navp_maxdist;
    pdata->gps_navp_loadarea = gps_navp_loadarea;
    pdata->gps_navp_latitude = gps_navp_latitude;
    pdata->gps_navp_longitude = gps_navp_longitude;
    pdata->gps_navp_latitude_raw = gps_navp_latitude_raw;
    pdata->gps_navp_longitude_raw = gps_navp_longitude_raw;

    pdata->gps_ref = gps_ref;
    pdata->gps_x = gps_x;
    pdata->gps_y = gps_y;

    // do not write coneseek_stop - just for logging purposes

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    
    LOGM_DEBUG(loggerIstro, "gps_writeData", "fix=" << gps_fix 
        << ", latitude=" << ioff(gps_latitude, 6) << ", longitude=" << ioff(gps_longitude, 6) 
        << ", latitude_raw=" << ioff(gps_latitude_raw, 6) << ", longitude_raw=" << ioff(gps_longitude_raw, 6) 
        << ", speed=" << ioff(gps_speed, 3) << ", course=" << ioff(gps_course, 2) 
        << ", gps_x=" << ioff(gps_x, 2) << ", gps_y=" << ioff(gps_y, 2)
        << ", ref=" << gps_ref  // << ", ref_latitude=" << ioff(gps_ref_latitude, 6) << ", ref_longitude=" << ioff(gps_ref_longitude, 6)
        << ", navp_dist=" << ioff(gps_navp_dist, 3) << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2) 
        << ", navp_dist_raw=" << ioff(gps_navp_dist_raw, 3) << ", navp_azimuth_raw=" << ioff(gps_navp_azimuth_raw, 2)
        << ", navp_maxdist=" << ioff(gps_navp_maxdist, 3) << ", navp_mindist=" << ioff(gps_navp_mindist, 3)
        << ", navp_loadarea=" << gps_navp_loadarea << ", navp_idx=" << gps_navp_idx 
        << ", navp_latitude=" << ioff(gps_navp_latitude, 6) << ", navp_longitude=" << ioff(gps_navp_longitude, 6) 
        << ", navp_latitude_raw=" << ioff(gps_navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(gps_navp_longitude_raw, 6) 
        << ", lastp_dist=" << ioff(gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(gps_lastp_azimuth, 2)
        << ", nmap_flags=" << nmap_flags << ", coneseek_stop=" << coneseek_stop);

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::gps_writeData", t);
#endif

    return 0;
}

int gps_readData(int &coneseek_stop, int &coneseek_intlen) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    coneseek_stop = pdata->coneseek_stop;
    coneseek_intlen = pdata->coneseek_intlen;
    
/*
    LOGM_DEBUG(loggerIstro, "gps_readData", "coneseek_stop=" << coneseek_stop);
*/
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::gps_readData", t);
#endif

    return 0;
}

const double NAVMAP_ROUTEFWD_DIST  = 10.0;  // set navigation point 10 metres ahead on route
const double NAVMAP_REROUTE_DIST  = 10.0;

void *gps_thread(void *parg)
{
    double gps_time;
    int gps_fix;
    double gps_latitude = ANGLE_NONE;
    double gps_longitude = ANGLE_NONE;
    double gps_latitude_raw = ANGLE_NONE;
    double gps_longitude_raw = ANGLE_NONE;
    double gps_speed;
    double gps_course;
    
    int gps_ref = 0;
    double gps_ref_latitude = ANGLE_NONE;
    double gps_ref_longitude = ANGLE_NONE;
    
    double gps_x = 0;
    double gps_y = 0;

    // previous gps fix position
    double lastp_latitude = ANGLE_NONE;
    double lastp_longitude = ANGLE_NONE;
    double gps_lastp_dist = -1;
    double gps_lastp_azimuth = ANGLE_NONE;

    // next navigation point
    int    navp_idx = -1;
    double navp_latitude = ANGLE_NONE;
    double navp_longitude = ANGLE_NONE;
    double navp_latitude_raw = ANGLE_NONE;
    double navp_longitude_raw = ANGLE_NONE;
    int    navp_loadarea = NAVIGATION_AREA_NONE;         // what loading action should be indicated now?
    int    navp_loadarea_next = NAVIGATION_AREA_NONE;    // after passing this point what loading action should be indicated?
    double gps_navp_dist = -1;
    double gps_navp_dist_raw = -1;
    double gps_navp_azimuth = ANGLE_NONE;
    double gps_navp_azimuth_raw = ANGLE_NONE;
    double gps_navp_maxdist = -1;
    double gps_navp_mindist = -1;
    
#ifndef ISTRO_NAVMAP_NO_ROUTING
    // navigation map variables
    int    nmap_np_idx = -1;    // last navigation point used for route planning
#endif

    int    coneseek_stop = -1;
    int    coneseek_intlen = -1;
        
    double t;
    int result = 0;
    
    LOG_THREAD_NAME("gps");
    LOGM_INFO(loggerIstro, "gps_thread", "msg=\"start\"");

    try {

    if (conf.useNavigation) {
        navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude_raw, navp_longitude_raw, navp_loadarea_next, navp_idx);
        navp_latitude = navp_latitude_raw;
        navp_longitude = navp_longitude_raw;
    }
  
    while ((!thread_testcancel()) && conf.useGPSDevice) {
        t = timeBegin();

        if ((gps_fix = gps.getData(gps_latitude_raw, gps_longitude_raw, gps_speed, gps_course)) < 0) {
            result = -2;
            break;
        }

        gps_time = timeBegin();

        if (gps_readData(coneseek_stop, coneseek_intlen) < 0) {
            result = -3;
            break;
        }
        
        gps_ref = navigation_ref_get(gps_ref_latitude, gps_ref_longitude);
        
        // reference point, gps_x/y
        if (gps_fix > 0) {
            if (gps_ref <= 0) {
                //gps_ref = navigation_ref_set(gps_latitude, gps_longitude);
                LOGM_ERROR(loggerIstro, "gps_thread", "msg=\"error: reference point not initialized!\"")
                result = -1;
                break;
            }
            navigation_getXY(gps_latitude_raw, gps_longitude_raw, gps_x, gps_y);
        } else {
            gps_latitude_raw = ANGLE_NONE;
            gps_longitude_raw = ANGLE_NONE;
        }
        gps_latitude = gps_latitude_raw;
        gps_longitude = gps_longitude_raw;

        // navigation
        if (gps_fix > 0) {
            // calculate distance and azimuth to last known GPS position
            if ((lastp_latitude < ANGLE_OK) && (lastp_longitude < ANGLE_OK)) {         
                geoCalc.getDist(lastp_latitude, lastp_longitude, gps_latitude, gps_longitude, gps_lastp_dist, gps_lastp_azimuth);
            } else {
                gps_lastp_dist = -1;
                gps_lastp_azimuth = ANGLE_NONE;
            }
            
            // calculate distance and azimuth to next navigation point, check if it was passed
            if (conf.useNavigation) {
                // wait for unknown navigation coordinates - have to be scanned from a QR-code
                if ((navp_idx >= 0) && ((navp_latitude_raw >= ANGLE_OK) || (navp_longitude_raw >= ANGLE_OK))) {
                    navigation_point_get(navp_idx, navp_latitude_raw, navp_longitude_raw);  // if > 0) { ... }
                    navp_latitude = navp_latitude_raw;
                    navp_longitude = navp_longitude_raw;
                }
                if ((navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                    geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw, navp_longitude_raw, gps_navp_dist_raw, gps_navp_azimuth_raw);
                    gps_navp_dist = gps_navp_dist_raw;
                    gps_navp_azimuth = gps_navp_azimuth_raw;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth1...\"");
                    int point_passed = gps_navp_dist_raw < NAVIGATION_DISTANCE_THRESHOLD;
                    // do we need to approach this point very closely (un/loading area)?
                    int navp_approach = navigation_approach(conf.navigationPath, navigationPathPos-2);
                    if (point_passed && navp_approach) {
                        // ak sme vo vzdialenosti do 10m a vidime kuzel, tak ignorujeme GPS vzdialenost 
                        // a cakame na priblizenie dostatocne blizko ku kuzelu
                        if (coneseek_intlen < 0) {
                        // wait for passing the second threshold or detecting that the distance is greater than the previous one                            
                            point_passed = (gps_navp_dist_raw < NAVIGATION_DISTANCE_THRESHOLD2) || 
                                           (gps_navp_dist_raw >= gps_navp_mindist + 1.5);
                        } else {
                            point_passed = (coneseek_stop > 0);
                        }
                    }
                    if (point_passed) {
                        LOGM_INFO(loggerIstro, "gps_thread", "msg=\"navigation point passed!\", pos=" << navigationPathPos
                            << ", name=\"" << conf.navigationPath[navigationPathPos-2] << conf.navigationPath[navigationPathPos-1] << "\""
                            << ", navp_dist=" << ioff(gps_navp_dist_raw, 3)
                            << ", navp_mindist=" << ioff(gps_navp_mindist, 3)
                            << ", navp_approach=" << navp_approach << ", coneseek_stop=" << coneseek_stop);
                        navp_loadarea = navp_loadarea_next;
                        navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude_raw, navp_longitude_raw, navp_loadarea_next, navp_idx);
                        navp_latitude = navp_latitude_raw;
                        navp_longitude = navp_longitude_raw;
                        gps_navp_maxdist = -1;
                        gps_navp_mindist = -1;
                        if ((navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                            geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw, navp_longitude_raw, gps_navp_dist_raw, gps_navp_azimuth_raw);
                            gps_navp_dist = gps_navp_dist_raw;
                            gps_navp_azimuth = gps_navp_azimuth_raw;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth2...\"");
                        } else {
                            gps_navp_dist = gps_navp_dist_raw = -1;
                            gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth3...\"");
                        }
                    }
                //printf("gps_thread: geoCalc.getDist(): pos=%d, dist=%f, azimuth=%f\n", navigationPathPos, gps_navp_dist, gps_navp_azimuth);
                } else {
                    gps_navp_dist = gps_navp_dist_raw = -1;
                    gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth4...\"");
                }
            } else {
                gps_navp_dist = gps_navp_dist_raw = -1;
                gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth5...\"");
            }
            
            lastp_latitude = gps_latitude;
            lastp_longitude = gps_longitude;
        } else {
            gps_navp_dist = gps_navp_dist_raw = -1;
            gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
//LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth6...\"");
            LOGM_WARN(loggerIstro, "gps_thread", "no gps-fix!");
        }
        
        // calculate maxdist
        if (gps_navp_dist_raw >= 0) {
            if ((gps_navp_maxdist < 0) || (gps_navp_maxdist < gps_navp_dist_raw)) {
                gps_navp_maxdist = gps_navp_dist_raw;
            }
            if ((gps_navp_mindist < 0) || (gps_navp_mindist > gps_navp_dist_raw)) {
                gps_navp_mindist = gps_navp_dist_raw;
            }
        }
        
        // navigation point changed? recalculate route
        int nmap_flags = 0;
#ifndef ISTRO_NAVMAP_NO_ROUTING
        if (nmap_np_idx != navp_idx) {
            if ((navp_idx >= 0) && (gps_fix > 0) && (navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                nmap_flags += 20;
                nmap_np_idx = navp_idx;
                // plan route 
                LOGM_INFO(loggerIstro, "gps_thread", "msg=\"nmap: planning new route...\", nmap_np_idx=" << nmap_np_idx << ", fix=" << gps_fix
                    << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                    << ", navp_latitude_raw=" << ioff(navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw, 6));
                navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw, navp_longitude_raw);
            } else {    // if (navp_idx < 0) 
                if (nmap_np_idx >= 0) {
                    nmap_flags += 40;
                    nmap_np_idx = -1;
                    LOGM_INFO(loggerIstro, "gps_thread", "msg=\"nmap: deleting route...\", nmap_np_idx=" << nmap_np_idx);
                    navmap_plan_delete();
                }
            }            
        }
        // route matching: replace gps and navigation point coordinates with points on route
        if ((nmap_np_idx >= 0) && (gps_fix > 0)) {
            int sidx = -1;
            double llat, llon;
            
            nmap_flags += 1;
            
            double dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);
            
            if (dd2 > NAVMAP_REROUTE_DIST * NAVMAP_REROUTE_DIST) {
                nmap_flags += 10;
                LOGM_INFO(loggerIstro, "gps_thread", "msg=\"nmap: rerouting...\", dd2=" << ioff(dd2, 3) 
                    << ", nmap_np_idx=" << nmap_np_idx << ", fix=" << gps_fix
                    << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                    << ", navp_latitude_raw=" << ioff(navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw, 6));
                navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw, navp_longitude_raw);
                
                dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);
            }
        
            if ((sidx >= 0) && (llat < ANGLE_OK) && (llon < ANGLE_OK)) {
                nmap_flags += 2;
                gps_latitude = llat;
                gps_longitude = llon; 
                navigation_getXY(gps_latitude, gps_longitude, gps_x, gps_y);
            }

            double nlat, nlon;
            
            int res = navmap_routeFwd(gps_latitude, gps_longitude, NAVMAP_ROUTEFWD_DIST, nlat, nlon);
            
            if ((res >= 0) && (nlat < ANGLE_OK) && (nlon < ANGLE_OK)) {
                nmap_flags += 4;
                double ndist, nazimuth;
                navp_latitude = nlat;
                navp_longitude = nlon;
                geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude, navp_longitude, ndist, nazimuth);
                // distance to navigation point is real (could be replaced with route length), azimuth is changed 
             /* gps_navp_dist = ndist;   */
                gps_navp_azimuth = nazimuth;
#if defined(ISTRO_MAP_BA_FEISTU) || defined(ISTRO_MAP_BA_FEISTU2)
if ((navigationPathPos == 2) and (ndist < 5)) {
int fixme_navp_azimuth;
gps_navp_azimuth = 1;
LOGM_DEBUG(loggerIstro, "gps_thread", "msg=\"navp_azimuth hotfix - fixme...\"");
}
#else
#if defined(ISTRO_MAP_PISEK_PALSADY) || defined(ISTRO_MAP_PISEK_PALSADY2)
if ((navigationPathPos == 2) and (ndist < 5)) {
int fixme_navp_azimuth2;
gps_navp_azimuth = -66;
LOGM_DEBUG(loggerIstro, "gps_thread", "msg=\"navp_azimuth hotfix2 - fixme...\"");
}
#else
int fixme_navp_azimuth3;
#endif
#endif
/*
LOGM_INFO(loggerIstro, "gps_thread", "msg=\"gps_navp_azimuth7...\", nlat=" << ioff(nlat, 6) 
    << ", nlon=" << ioff(nlon, 6)
    << ", ndist=" << ioff(ndist, 3)
    << ", sidx=" << sidx
    << ", navpp=" << navigationPathPos
    << ", navp_dist=" << ioff(gps_navp_dist, 3) << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2) 
    << ", navp_dist_raw=" << ioff(gps_navp_dist_raw, 3) << ", navp_azimuth_raw=" << ioff(gps_navp_azimuth_raw, 2)
    << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
    << ", gps_latitude=" << ioff(gps_latitude, 6) << ", gps_longitude=" << ioff(gps_longitude, 6)
    << ", navp_latitude=" << ioff(navp_latitude, 6) << ", navp_longitude=" << ioff(navp_longitude, 6)
    << ", navp_latitude_raw=" << ioff(navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw, 6));
*/
            }
        }
#endif        

        // store last N gps raw positions 
        if ((gps_fix > 0) && (gps_longitude_raw < ANGLE_OK) && (gps_latitude_raw < ANGLE_OK)) {
            if (gps_pt_cnt < GPS_POINT_NUM) {
                gps_pt[gps_pt_cnt].style[0] = 0;        // initialize the new point
                gps_pt[gps_pt_cnt + 1].style[0] = 0;    // mark the stopper 
                gps_pt_cnt++;
            }
            
            for(int i = gps_pt_cnt - 1; i >= 1; i--) {
                gps_pt[i] = gps_pt[i - 1];
            }
            gps_pt[0].name[0] = 0;
            gps_pt[0].desc[0] = 0;
            strcpy(gps_pt[0].style, "g");
            gps_pt[0].longitude = gps_longitude_raw;
            gps_pt[0].latitude = gps_latitude_raw;
        }
        
        timeEnd("istro::gps_thread.capture", t);

        if (gps_writeData(gps_time, gps_fix, gps_latitude, gps_longitude, gps_latitude_raw, gps_longitude_raw, 
                gps_speed, gps_course, 
                gps_lastp_dist, gps_lastp_azimuth, 
                gps_navp_dist, gps_navp_dist_raw, gps_navp_azimuth, gps_navp_azimuth_raw, 
                gps_navp_maxdist, gps_navp_mindist, 
                navp_latitude, navp_longitude, navp_latitude_raw, navp_longitude_raw, navp_loadarea, navp_idx,
                gps_ref, gps_x, gps_y, gps_ref_latitude, gps_ref_longitude, nmap_flags, coneseek_stop) < 0) {
            result = -4;
            break;
        }
        
        LOGM_DEBUG(loggerIstro, "gps_thread", "msg=\"data updated\"");
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "gps_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }
    
    if (result >= 0) {
        LOGM_INFO(loggerIstro, "gps_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "gps_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

int ahrs_writeData(int ahrs_roll, double ahrs_pitch, double ahrs_yaw) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ahrs_roll = ahrs_roll; 
    pdata->ahrs_pitch = ahrs_pitch; 
    pdata->ahrs_yaw = ahrs_yaw;
    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    
    LOGM_DEBUG(loggerIstro, "ahrs_writeData", "ahrs_roll=" << ioff(ahrs_roll, 2) << ", ahrs_pitch=" << ioff(ahrs_pitch, 2) << ", ahrs_yaw=" << ioff(ahrs_yaw, 2));
    
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::ahrs_writeData", t);
#endif

    return 0;
}

void *ahrs_thread(void *parg)
{
    double ahrs_roll; 
    double ahrs_pitch; 
    double ahrs_yaw;

    double t;
    int result = 0;

    LOG_THREAD_NAME("ahrs");
    LOGM_INFO(loggerIstro, "ahrs_thread", "msg=\"start\"");

    try {

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
            LOGM_DEBUG(loggerIstro, "ahrs_thread", "msg=\"data updated\"");
        }
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "ahrs_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }
    
    if (result >= 0) {
        LOGM_INFO(loggerIstro, "ahrs_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "ahrs_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

int ctrlBoard_writeData(double ctrlb_time1, int ctrlb_state, int ctrlb_ircv, double ctrlb_ircv500, int ctrlb_angle, int ctrlb_velocity, int ctrlb_loadd,
        double ctrlb_time2, double ctrlb_euler_x, double ctrlb_euler_y, double ctrlb_euler_z, 
        int ctrlb_calib_gyro, int ctrlb_calib_accel, int ctrlb_calib_mag, 
        int data1, int data2) 
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    if (data1) {
        pdata->ctrlb_time1 = ctrlb_time1;
        pdata->ctrlb_state = ctrlb_state;
        pdata->ctrlb_ircv = ctrlb_ircv;
        pdata->ctrlb_ircv500 = ctrlb_ircv500;
        pdata->ctrlb_angle = ctrlb_angle; 
        pdata->ctrlb_velocity = ctrlb_velocity;
        pdata->ctrlb_loadd = ctrlb_loadd;
    }
    if (data2) {
        pdata->ctrlb_time2 = ctrlb_time2;
        pdata->ctrlb_euler_x = ctrlb_euler_x;
        pdata->ctrlb_euler_y = ctrlb_euler_y;
        pdata->ctrlb_euler_z = ctrlb_euler_z;
        pdata->ctrlb_calib_gyro = ctrlb_calib_gyro; 
        pdata->ctrlb_calib_accel = ctrlb_calib_accel;
        pdata->ctrlb_calib_mag = ctrlb_calib_mag;
    }

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    
    if (data1 && data2) {
        LOGM_DEBUG(loggerIstro, "ctrlBoard_writeData", "state=" << ctrlb_state << 
            ", ctrlb_ircv=" << ctrlb_ircv << ", ircv500=" << ioff(ctrlb_ircv500, 2) << 
            ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity << ", ctrlb_loadd=" << ctrlb_loadd <<
            ", euler_x=" << ioff(ctrlb_euler_x, 2) << ", euler_y=" << ioff(ctrlb_euler_y, 2) << ", euler_z=" << ioff(ctrlb_euler_z, 2) << 
            ", calib_gyro=" << ctrlb_calib_gyro << ", calib_accel=" << ctrlb_calib_accel << ", calib_mag=" << ctrlb_calib_mag);
    } else
    if (data1 && !data2) {
        LOGM_DEBUG(loggerIstro, "ctrlBoard_writeData", "state=" << ctrlb_state <<
            ", ctrlb_ircv=" << ctrlb_ircv << ", ircv500=" << ioff(ctrlb_ircv500, 2) << 
            ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity << ", ctrlb_loadd=" << ctrlb_loadd);
    } else
    if (!data1 && data2) {
        LOGM_DEBUG(loggerIstro, "ctrlBoard_writeData", "euler_x=" << ioff(ctrlb_euler_x, 2) <<
            ", euler_y=" << ioff(ctrlb_euler_y, 2) << ", euler_z=" << ioff(ctrlb_euler_z, 2) <<
            ", calib_gyro=" << ctrlb_calib_gyro << ", calib_accel=" << ctrlb_calib_accel <<
            ", calib_mag=" << ctrlb_calib_mag);
    }

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::ctrlBoard_writeData", t);
#endif

    return 0;
}

void *ctrlBoard_thread(void *parg)
{
    double ctrlb_time1;    
    int    ctrlb_state;       // stav robota 1-STOP, 2-FWD, 5-OBST
    double ctrlb_heading;     // heading z kompasu (nepouziva sa)
    int    ctrlb_ircv;        // inkrement z IRC (Incremental rotary encoders) v casovom intervale, moze poslat 1 a viacej impulzov
    double ctrlb_ircv500;     // suma ircv za poslednych 500ms
    int    ctrlb_angle;       // aktualne nastavenie serva riadenia
    int    ctrlb_velocity;    // aktualne nastavenie motora
    int    ctrlb_loadd;       // detekcia nakladu (load detection)
    int    ctrlb_cbtime;      // cas v milisekundach: millis() % 10000
    int    ctrlb_ulsd1;       // lavy ultrazvuk v cm
    int    ctrlb_ulsd2;       // stredny/predny ultrazvuk v cm
    int    ctrlb_ulsd3;       // pravy ultrazvuk v cm
    int    ctrlb_ulsd4;       // zadny ultrazvuk v cm
    int    ctrlb_ulsd5;       // zadny ultrazvuk v cm

    double ctrlb_time2;
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int    ctrlb_calib_gyro;
    int    ctrlb_calib_accel;
    int    ctrlb_calib_mag;

    int res, res2;
    int result = 0;
    
    LOG_THREAD_NAME("ctrlBoard");
    LOGM_INFO(loggerIstro, "ctrlBoard_thread", "msg=\"start\"");

    try {

    while ((!thread_testcancel()) && (conf.useControlBoard || conf.useControlBoard2)) {
#ifdef CTRLBOARD_LOG_TRACE0
        double t = timeBegin();
#endif

        ctrlb_time1 = timeBegin();
        if ((res = ctrlBoard.getServoData(ctrlb_state, ctrlb_heading, ctrlb_ircv, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd, ctrlb_cbtime, 
                      ctrlb_ulsd1, ctrlb_ulsd2, ctrlb_ulsd3, ctrlb_ulsd4, ctrlb_ulsd5)) < 0) {
            result = -1;
            break;
        }

        ctrlb_time2 = timeBegin();
        if ((res2 = ctrlBoard.getImuData(ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag)) < 0) {
            result = -2;
            break;
        }

#ifdef CTRLBOARD_LOG_TRACE0
        timeEnd("istro::ctrlBoard_thread.capture", t);
#endif
        if ((res > 0) || (res2 > 0)) {
            if (ctrlBoard_writeData(ctrlb_time1, ctrlb_state, ctrlb_ircv, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd,
                    ctrlb_time2, ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag, 
                    res > 0, res2 > 0) < 0) {
                result = -3;
                break;
            }
#ifdef CTRLBOARD_LOG_TRACE0
            LOGM_DEBUG(loggerIstro, "ctrlBoard_thread", "msg=\"data updated\"");
#endif
        } else {
            // sleep because we are performing asynchronous reads (expecting data 10 times per second)
            msleep(1); //);
        }
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "ctrlBoard_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }

    if (result >= 0) {
        LOGM_INFO(loggerIstro, "ctrlBoard_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "ctrlBoard_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

int capture_readData(long &image_number, int &process_dir, 
        int &process_ref, double &process_x, double &process_y, double &process_yaw) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    image_number = pdata->image_number;
    process_dir = pdata->process_dir;
    process_ref = pdata->process_ref;
    process_x = pdata->process_x;
    process_y = pdata->process_y;
    process_yaw = pdata->process_yaw;
    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    
    LOGM_DEBUG(loggerIstro, "capture_readData", "image_number=" << image_number << ", process_dir=" << process_dir
        << ", process_ref=" << process_ref << ", process_x=" << ioff(process_x, 2) << ", process_y=" << ioff(process_y, 2)
        << ", process_yaw=" << ioff(process_yaw, 2));
    
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::capture_readData", t);
#endif

    return 0;
}


void *capture_camera_thread(void *parg)
{
    double t;
    int result = 0;
    
    DataSet *pdata;

    LOG_THREAD_NAME("capture_camera");
    LOGM_INFO(loggerIstro, "capture_camera_thread", "msg=\"start\"");

    try {

    while ((!thread_testcancel()) && conf.useCamera) {
        t = timeBegin();
        pdata = (DataSet *)threads.getData(THDATA_STATE_NEW, THDATA_STATE_CAMERA_CAPTURING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::capture_camera_thread.wait", t);

        /* read Shared data */
        if (capture_readData(pdata->image_number, pdata->process_dir, pdata->process_ref, pdata->process_x, pdata->process_y, pdata->process_yaw) < 0) {
            result = -2;
            break;
        }

        t = timeBegin();
        if (conf.useCamera) {
            if (camera.getFrame(pdata->camera_img) < 0) {
                threads.setData(pdata, THDATA_STATE_NEW, -1);
                LOGM_ERROR(loggerIstro, "capture_camera_thread", "camera.getFrame() failed!");
                //result = -3;
                //break;
                msleep(10);
                continue;
            }
        }
        timeEnd("istro::capture_camera_thread.capture", t);        

        threads.setData(pdata, THDATA_STATE_CAMERA_CAPTURED, 1);
        LOGM_DEBUG(loggerIstro, "capture_camera_thread", "msg=\"data captured\"");
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "capture_camera_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }

    if (result >= 0) {
        LOGM_INFO(loggerIstro, "capture_camera_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "capture_camera_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

void *capture_lidar_thread(void *parg)
{
    double t;
    int result = 0;
    
    DataSet *pdata;

    LOG_THREAD_NAME("capture_lidar");
    LOGM_INFO(loggerIstro, "capture_lidar_thread", "msg=\"start\"");

    try {

    while ((!thread_testcancel()) && conf.useLidar) {
        t = timeBegin();        
        pdata = (DataSet *)threads.getData(THDATA_STATE_NEW, THDATA_STATE_LIDAR_CAPTURING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::capture_lidar_thread.wait", t);
        /* read Shared data */
        if (capture_readData(pdata->image_number, pdata->process_dir, pdata->process_ref, pdata->process_x, pdata->process_y, pdata->process_yaw) < 0) {
            result = -2;
            break;
        }
        t = timeBegin();
        if (conf.useLidar) {
            if (lidar.getData(pdata->lidar_data, pdata->lidar_data_cnt) < 0) {
                threads.setData(pdata, THDATA_STATE_NEW, -1);
                LOGM_INFO(loggerIstro, "capture_lidar_thread", "msg=\"lidar.getData() failed!\"");
                result = -3;
                break;
            }
        }
        timeEnd("istro::capture_lidar_thread.capture", t);

        threads.setData(pdata, THDATA_STATE_LIDAR_CAPTURED, 1);
        LOGM_DEBUG(loggerIstro, "capture_lidar_thread", "msg=\"data captured\"");
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "capture_lidar_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }

    if (result >= 0) {
        LOGM_INFO(loggerIstro, "capture_lidar_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "capture_lidar_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

const int DMAP_MIN_INTERVAL_LENGTH = 20;

static volatile int vision_thread_qrscan_enabled = 0;

int vision_thread_qrscan_enable(void)
{
    if (vision_thread_qrscan_enabled) {
        return 0;
    }
    
    vision_thread_qrscan_enabled = 1;
    LOGM_INFO(loggerIstro, "vision_thread_qrscan_enable", "msg=\"QRScan processing enabled!\"");
    return 1;
}

int vision_thread_qrscan_disable(void)
{
    if (!vision_thread_qrscan_enabled) {
        return 0;
    }
    
    vision_thread_qrscan_enabled = 0;
    LOGM_INFO(loggerIstro, "vision_thread_qrscan_disable", "msg=\"QRScan processing disabled!\"");
    return 0;
}

void *vision_thread(void *parg)
{
    double t;
    int result = 0;
    
    DataSet *pdata;

    LOG_THREAD_NAME("vision");
    LOGM_INFO(loggerIstro, "vision_thread", "msg=\"start\"");

    try {
 
    while ((!thread_testcancel()) && conf.useCamera) {
        t = timeBegin();
        pdata = (DataSet *)threads.getData(THDATA_STATE_CAMERA_CAPTURED, THDATA_STATE_VISION_PROCESSING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::vision_thread.wait", t);

        /* camera_capture_thread already prepared pdata: camera_img + image_number, process_dir, process_ref, process_x, process_y, process_yaw */

        t = timeBegin();
        
        if (conf.useQRScan && vision_thread_qrscan_enabled) {
            double t2 = timeBegin();
            qrscan.scanGeo(pdata->camera_img, pdata->qrscan_latitude, pdata->qrscan_longitude);
            timeEnd("istro::vision_thread.qrscan_process", t2);
        } else {
            pdata->qrscan_latitude  = ANGLE_NONE;
            pdata->qrscan_longitude = ANGLE_NONE;
        }
        
        vision.eval(pdata->camera_img, pdata->vision_markers, pdata->vision_markersIM, pdata->vision_epweigth, pdata->vision_elweigth, pdata->vision_dmap);
        //pdata->vision_dmap.print("vision_dmap");

        pdata->vision_dmap.find(DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->vision_angle_min, pdata->vision_angle_max);
        LOGM_INFO(loggerIstro, "vision_thread", "dmap_camera: vision_angle_min=" << pdata->vision_angle_min 
            << ", vision_angle_max=" << pdata->vision_angle_max << ", process_dir=" << pdata->process_dir 
            << ", qrscan_latitude=" << ioff(pdata->qrscan_latitude, 6) << ", qrscan_longitude=" << ioff(pdata->qrscan_longitude, 6));
        timeEnd("istro::vision_thread.process", t);

        threads.setData(pdata, THDATA_STATE_VISION_PROCESSED, 1);
        LOGM_DEBUG(loggerIstro, "vision_thread", "msg=\"processed\"");
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "vision_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }

    if (result >= 0) {
        LOGM_INFO(loggerIstro, "vision_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "vision_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

void update_angle_interval(double &angle_min, double &angle_max, double angle)
{
    if ((angle_min >= ANGLE_OK) || (angle_max >= ANGLE_OK)) {
        angle_min = angle_max = angle;
        return;
    } 
    
    if ((angle_min <= angle) && (angle <= angle_max)) {
        return;
    }

    if (angle < angle_min) {
        double angle2 = angle + 360;
        
        if (angle2 <= angle_max) {  // ((angle_min <= angle2) && 
            return;
        }
        
        // angle2 > angle_max
        double dt1 = angle_min - angle;
        double dt2 = angle2 - angle_max;
        if (dt1 < dt2) {
            angle_min = angle;                    // example: (300, 310), 210 -> (210, 310)
        } else {
            angle_max = angle2;                 // example: (300, 310),  10 -> (300, 370)
        }
        return;
    }

    if (angle > angle_max) {
        double angle2 = angle - 360;
        
        if (angle_min <= angle2) {  // ((angle2 <= angle_max) && 
            return;
        }
        
        // angle2 < angle_min
        double dt1 = angle_min - angle2;
        double dt2 = angle - angle_max;
        if (dt1 < dt2) {
            angle_min = angle2;                 // example: (100, 110), 350 -> (-10, 110)
        } else {
            angle_max = angle;                    // example: (100, 110), 120 -> (100, 120)
        }
        return;
    }
}

void update_speed_interval(double &speed_min, double &speed_max, double speed)
{
    if ((speed_min < 0) || (speed_max < 0)) {
        speed_min = speed_max = speed;
        return;
    }
    
    if (speed_max < speed) {
        speed_max = speed;
    } else
    if (speed_min > speed) {
        speed_min = speed;
    }
}

const int CALIB_TIMEOUT         = 18000000;
const int CALIB_TIME1_SPEED_CHK =   2500;
const int CALIB_TIME2_ANGLE_CHK =   3500;
const int CALIB_TIME3_FINISH    =   7000;

const int CALIB_RESET_REASON_OBSTACLE = 1;
const int CALIB_RESET_REASON_LOADAREA = 2;

const double CALIB_GPS_SPEED_MIN_VALUE =   0.2;
const double CALIB_GPS_COURSE_MAX_DIFF =  15.0;
const double CALIB_IRCV500_MIN_VALUE   =   5.0;
const double CALIB_YAW_MAX_DIFF        =   5.0;

const double CALIB_NAV_ANGLE_MIN       = -35.0;
const double CALIB_NAV_ANGLE_MAX       = +35.0;

const double CALIB_NAVP_DIST1          =  20.0;  // do not perform calibration 20 metres before navigation point (navigation point will be detected in distance 10 metres)
const double CALIB_NAVP_DIST2          =  15.0;  // do not perform calibration 15 metres after navigation point

typedef struct { 
    int    ok;
    double azimuth;
    double yaw;
    
    int    first;           // first calibration needs to be performed?
    int    navp_ok;         // distance to navigation point is OK -> calibration could be performed at current position
    int    nav_angle_ok;    // course towards next goal is OK

    double time;
    double gps_course_min;
    double gps_course_max;    
    double yaw_min;
    double yaw_max;
    double gps_speed_min;
    double gps_speed_max;   
} calib_t;

void calib_init(calib_t &calib) 
{    
    calib.azimuth = conf.calibGpsAzimuth; 
    calib.yaw     = conf.calibImuYaw;

    calib.ok = (calib.azimuth < (int)ANGLE_OK) && (calib.yaw < (int)ANGLE_OK);
    calib.first = !calib.ok;
    calib.navp_ok = -1;
    calib.nav_angle_ok = -1;

    calib.time = -1;
    calib.gps_course_min = ANGLE_NONE;
    calib.gps_course_max = ANGLE_NONE;    
    calib.yaw_min = ANGLE_NONE;
    calib.yaw_max = ANGLE_NONE;
    calib.gps_speed_min = -1;
    calib.gps_speed_max = -1;    
}

int calib_reset(calib_t &calib, const string &log_reason, const string &log_str)
{
    // reset calibration time, if obstacle appeared during calibration
    if ((!calib.ok) && (calib.time >= 0)) {
        calib.time = -1;
        
        LOGM_INFO(loggerIstro, "calib_reset", "msg=\"calibration interrupted!\", reason=\"" << log_reason 
                << "\", calib.ok=" << calib.ok << log_str);
    }
    
    return 0;
}

int calib_process(calib_t &calib, double nav_angle, int process_angle_min, int process_angle_max,
        double gps_course, double yaw, double gps_speed, double navp_dist, double navp_maxdist, double ctrlb_ircv500)
{
    // check calibration timeout
    if (calib.ok) {
        if (calib.time < 0) {
            calib.time = timeBegin();
        } 
        if (timeDelta(calib.time) > CALIB_TIMEOUT) {
            calib.ok = 0;
            calib.time = -1;
            calib.navp_ok = -1;
            calib.nav_angle_ok = -1;
            LOGM_INFO(loggerIstro, "calib_process", "msg=\"calibration needed (timeout)!\", calib.ok=" << calib.ok);
        }
    }

    // reset calibration time, if obstacle appeared during calibration
    // if ((!noobstacle) && (!calib.ok) && (calib.time >= 0)) {
    // performed using function calib_reset()

    // noobstacle != 0   -> no obstacle seen, calibration could be performed (at this point)

    // no calibration needed
    if (calib.ok) {
        return 0;
    }
    
    // check "nav_angle"
    int nav_angle_ok = -1;
    if (nav_angle < ANGLE_OK) {
        nav_angle_ok = (nav_angle >= CALIB_NAV_ANGLE_MIN) && (nav_angle <= CALIB_NAV_ANGLE_MAX);
        // before first calibration the nav_angle_ok will always be true (calibration must be performed!)
        if ((!nav_angle_ok) && (calib.first)) {
            nav_angle_ok = 1;
        }
    }
    if (nav_angle_ok != calib.nav_angle_ok) {
        LOGM_DEBUG(loggerIstro, "calib_process", "msg=\"calibration - nav_angle_ok changed...\", calib.ok=" << calib.ok
                    << ", calib.nav_angle_ok=" << nav_angle_ok << ", calib.nav_angle_ok_old=" << calib.nav_angle_ok
                    << ", calib.first=" << calib.first << ", nav_angle=" << ioff(nav_angle, 2));
        calib.nav_angle_ok = nav_angle_ok;
    }

    if (calib.nav_angle_ok == 0) {
        calib.time = -1;
        return 0;
    }
    
    // is distance to next navigation point ok?
    if ((navp_dist >= 0) && (navp_maxdist >= 0)) {
        int navp_ok = (navp_dist > CALIB_NAVP_DIST1) && ((navp_maxdist - navp_dist) > CALIB_NAVP_DIST2);
        // before first calibration the navp_ok will always be true (calibration must be performed!)
        if ((!navp_ok) && (calib.first)) {
            navp_ok = 1;
        }
        if (navp_ok != calib.navp_ok) {
            if ((!navp_ok) && (!calib.ok) && (calib.time >= 0)) {
                stringstream ss;
                ss << ", calib.navp_ok=" << navp_ok << ", calib.navp_ok_old=" << calib.navp_ok
                    << ", calib.first=" << calib.first << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_maxdist=" << ioff(navp_maxdist, 3);
                calib_reset(calib, "close to navigation point", ss.str());
            } else {
                LOGM_DEBUG(loggerIstro, "calib_process", "msg=\"calibration - navp_ok changed...\", calib.ok=" << calib.ok
                    << ", calib.navp_ok=" << navp_ok << ", calib.navp_ok_old=" << calib.navp_ok
                    << ", calib.first=" << calib.first 
                    << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_maxdist=" << ioff(navp_maxdist, 3));            
            }
        }
        calib.navp_ok = navp_ok;
    } else {
        calib.navp_ok = -1;
    }
    
    if (calib.navp_ok == 0) {
        calib_reset(calib, "navp_ok", "");
        return 0;
    }

    // start time calculation if not initialized
    if (calib.time < 0) {
        calib.time = timeBegin();
        calib.gps_course_min = calib.gps_course_max = ANGLE_NONE;
        calib.yaw_min = calib.yaw_max = ANGLE_NONE;
        calib.gps_speed_min = calib.gps_speed_max = -1;    
        LOGM_INFO(loggerIstro, "calib_process", "msg=\"calibration start!\", calib.ok=" << calib.ok 
            << ", calib.navp_ok=" << calib.navp_ok << ", calib.nav_angle_ok=" << calib.nav_angle_ok);
    } 

    // log all data checked during calibration
    LOGM_TRACE(loggerIstro, "calib_process", "msg=\"calibration in progress...\", calib.ok=" << calib.ok 
        << ", calib.navp_ok=" << calib.navp_ok << ", calib.nav_angle_ok=" << calib.nav_angle_ok << ", calib.first=" << calib.first 
        << ", gps_course=" << ioff(gps_course, 2) << ", yaw=" << ioff(yaw, 2) << ", gps_speed=" << ioff(gps_speed, 3)
        << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_maxdist=" << ioff(navp_maxdist, 3) << ", ircv500=" << ioff(ctrlb_ircv500, 2));

    // gps speed check
    if ((calib.time >= 0) && (timeDelta(calib.time) >= CALIB_TIME1_SPEED_CHK)) {
        update_speed_interval(calib.gps_speed_min, calib.gps_speed_max, gps_speed);
        if (gps_speed < CALIB_GPS_SPEED_MIN_VALUE) {
            stringstream ss;
            ss << ", gps_speed=" << ioff(gps_speed, 3) << ", gps_speed_min=" << ioff(calib.gps_speed_min, 3) << ", gps_speed_max=" << ioff(calib.gps_speed_max, 3);
            calib_reset(calib, "low gps speed", ss.str());
        }
        if (ctrlb_ircv500 < CALIB_IRCV500_MIN_VALUE) {
            stringstream ss;
            ss << ", ctrlb_ircv500=" << ioff(ctrlb_ircv500, 2);
            calib_reset(calib, "low ircv500", ss.str());
        }
    }

    // gps course and yaw check
    if ((calib.time >= 0) && (timeDelta(calib.time) >= CALIB_TIME2_ANGLE_CHK)) {
        update_angle_interval(calib.gps_course_min, calib.gps_course_max, gps_course);
        update_angle_interval(calib.yaw_min, calib.yaw_max, yaw);
        if ((calib.gps_course_max - calib.gps_course_min) > CALIB_GPS_COURSE_MAX_DIFF) {
            stringstream ss;
            ss << ", gps_course=" << ioff(gps_course, 3) << ", gps_course_min=" << ioff(calib.gps_course_min, 2) << ", gps_course_max=" << ioff(calib.gps_course_max, 2);
            calib_reset(calib, "gps_course difference", ss.str());
        }
        if ((calib.yaw_max - calib.yaw_min) > CALIB_TIME3_FINISH) {
            stringstream ss;
            ss << ", yaw=" << ioff(yaw, 2) << ", yaw_min=" << ioff(calib.yaw_min, 2) << ", yaw_max=" << ioff(calib.yaw_max, 2);
            calib_reset(calib, "yaw difference", ss.str());
        }
    }

    // finished?
    if ((calib.time >= 0) && (timeDelta(calib.time) >= 7000)) {
        calib.ok = 1;
        calib.first = 0;
        calib.azimuth = gps_course;
        calib.yaw = yaw;
        calib.time = -1;
        LOGM_INFO(loggerIstro, "calib_process", "msg=\"calibration finished!\", calib.ok=" << calib.ok
            << ", calib.yaw=" << ioff(calib.yaw, 2) << ", calib.azimuth=" << ioff(calib.azimuth, 2)
            << ", calib.delta=" << ioff(calib.azimuth - calib.yaw, 2)
            << ", gps_course_min=" << ioff(calib.gps_course_min, 2) << ", gps_course_max=" << ioff(calib.gps_course_max, 2)
            << ", yaw_min=" << ioff(calib.yaw_min, 2) << ", yaw_max=" << ioff(calib.yaw_max, 2)
            << ", gps_speed_min=" << ioff(calib.gps_speed_min, 3) << ", gps_speed_max=" << ioff(calib.gps_speed_max, 3));
    }

    return 1;
}

const int WRONGWAY_TIMEOUT      = 20000;
const int WRONGWAY_TIME1_STOP   =  4000;
const int WRONGWAY_TIME2_BACK   =  9000;
const int WRONGWAY_TIME3_STOP   = 13000;
const int WRONGWAY_BACKV_TO     =   500;     // change speed BACK / BACK+1 every 500ms

const double WRONGWAY_ANGLE_MIN  = -60.0;    // min/max difference between target direction and our heading (if exceeded wrongway-check will start)
const double WRONGWAY_ANGLE_MAX  = +60.0;
const double WRONGWAY_YAW0_DIFF  =  45.0;    // target direction must not change more than 45 degrees during wrongway-check

typedef struct { 
    int    ok;     // 1 = do check, 0 = go backward!

    double time;
    double yaw0;   // what direction should I turn to (0..360)
    double yaw0_min; 
    double yaw0_max;
    
    int    backv;
    double backv_to;  // change speed BACK / BACK+1
} wrongway_t ;

void wrongway_init(wrongway_t &wrongway)
{
    wrongway.ok = 1;
    wrongway.time = -1;
    wrongway.yaw0 = ANGLE_NONE;
    wrongway.yaw0_min = ANGLE_NONE;
    wrongway.yaw0_max = ANGLE_NONE;

    wrongway.backv = 0;
    wrongway.backv_to = -1;
}

int wrongway_reset(wrongway_t &wrongway, const string &log_reason)
{
    if ((wrongway.ok) && (wrongway.time >= 0)) {
        wrongway.time = -1;
        
        LOGM_INFO(loggerIstro, "wrongway_reset", "msg=\"wrongway-check interrupted!\", reason=\"" << log_reason 
                << "\", wrongway.ok=" << wrongway.ok);  // << log_str);
    }
    return 0;
}

int wrongway_check(wrongway_t &wrongway, double yaw, double angle, double navp_dist, double navp_maxdist)
// (angle > -180) && (angle <= 180)
{
    if (!wrongway.ok) {
        return 0;
    }
    
    if ((angle >= WRONGWAY_ANGLE_MIN) && (angle <= WRONGWAY_ANGLE_MAX)) {
        if (wrongway.time >= 0) {
            LOGM_INFO(loggerIstro, "wrongway_check", "msg=\"wrongway-check interrupted (angle is OK)!\", wrongway.ok=" << wrongway.ok 
                << ", angle=" << ioff(angle, 2));
        }
        wrongway.time = -1;
        return 0;
    }
    
    // is distance to next navigation point ok?
    if ((navp_dist >= 0) && (navp_maxdist >= 0)) {
        int navp_ok = (navp_dist > CALIB_NAVP_DIST1) && ((navp_maxdist - navp_dist) > CALIB_NAVP_DIST2);
        if (!navp_ok) {
            if (wrongway.time >= 0) {
                LOGM_INFO(loggerIstro, "wrongway_check", "msg=\"wrongway-check interrupted (navp_ok is FALSE)!\", wrongway.ok=" << wrongway.ok 
                    << ", navp_ok=" << navp_ok);
            }
            wrongway.time = -1;
            return 0;
        }
    }

    if (wrongway.time < 0) {
        wrongway.time = timeBegin();
        wrongway.yaw0_min = wrongway.yaw0_max = ANGLE_NONE;
        LOGM_INFO(loggerIstro, "wrongway_check", "msg=\"wrongway-check start!\", wrongway.ok=" << wrongway.ok);
    }

    double yaw0 = yaw + angle;
    while (yaw0 >= 360) {
        yaw0 -= 360; 
    }
    while (yaw0 < 0) {
        yaw0 += 360;
    }
    update_angle_interval(wrongway.yaw0_min, wrongway.yaw0_max, yaw0);
    
    if ((wrongway.yaw0_max - wrongway.yaw0_min) > WRONGWAY_YAW0_DIFF) {
        wrongway.time = -1;
        LOGM_INFO(loggerIstro, "wrongway_check", "msg=\"wrongway-check interrupted (yaw0_diff exceeded)!\", wrongway.ok=" << wrongway.ok 
            << ", wrongway.yaw0_min=" << ioff(wrongway.yaw0_min, 2) << ", wrongway.yaw0_max=" << ioff(wrongway.yaw0_max, 2));
        return 0;        
    }
    
    if (timeDelta(wrongway.time) >= WRONGWAY_TIMEOUT) {
        wrongway.ok = 0;
        wrongway.time = -1;
        wrongway.yaw0 = yaw0;
    
        LOGM_INFO(loggerIstro, "wrongway_check", "msg=\"wrongway detected!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << wrongway.yaw0 << ", yaw=" << ioff(yaw, 2) << ", angle=" << angle);
        return 1;
    }

    LOGM_TRACE(loggerIstro, "wrongway_check", "msg=\"wrongway-check in progress...\", wrongway.ok=" << wrongway.ok 
        << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2) << ", yaw0=" << ioff(yaw0, 2) 
        << ", wrongway.yaw0_min=" << ioff(wrongway.yaw0_min, 2) << ", wrongway.yaw0_max=" << ioff(wrongway.yaw0_max, 2));

    return 0;
}

int wrongway_force(wrongway_t &wrongway, double yaw, double angle)
// (angle > -180) && (angle <= 180)
{
    if (!wrongway.ok) {
        return 0;
    }

    double yaw0 = yaw + angle;
    while (yaw0 >= 360) {
        yaw0 -= 360; 
    }
    while (yaw0 < 0) {
        yaw0 += 360;
    }

    wrongway.ok = 0;
    wrongway.time = -1;
    wrongway.yaw0 = yaw0;

    LOGM_INFO(loggerIstro, "wrongway_force", "msg=\"wrongway forced!\", wrongway.ok=" << wrongway.ok
        << ", wrongway.yaw0=" << wrongway.yaw0 << ", yaw=" << ioff(yaw, 2) << ", angle=" << angle);
        
    return 1;
}

int wrongway_process(wrongway_t &wrongway, double yaw, int &process_angle, int &process_velocity, int &process_state)
{
    if (wrongway.ok) {
        return 0;
    }
    
    // change wrongway.backv value 0/1 every 500ms
    if (wrongway.backv_to < 0) {
        wrongway.backv_to = timeBegin();
    } else {
        if (timeDelta(wrongway.backv_to) >= WRONGWAY_BACKV_TO) {
            if (wrongway.backv == 0) {
                wrongway.backv = -1;
            } else {
                wrongway.backv = 0;
            }
            wrongway.backv_to = timeBegin();
        }
    }

    if (wrongway.time < 0) {
        wrongway.time = timeBegin();
        LOGM_INFO(loggerIstro, "wrongway_process", "msg=\"wrongway start!\", wrongway.ok=" << wrongway.ok);
    }
    if (timeDelta(wrongway.time) < WRONGWAY_TIME1_STOP) {
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstro, "wrongway_process", "msg=\"wrongway in progress (STOP1)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) 
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME2_BACK) {
        double angle = wrongway.yaw0 - yaw;
        while (angle > 180) {
            angle -= 360; 
        }
        while (angle <= -180) {
            angle += 360;
        }
        // we will go backwards -> we need oppsite angle 
        angle = -angle;
        // calculate process_angle
        process_angle = 90 - trunc(angle);
        if (process_angle < NAVIGATION_ANGLE_MIN) {
            process_angle = NAVIGATION_ANGLE_MIN;
        }
        if (process_angle > NAVIGATION_ANGLE_MAX) {
            process_angle = NAVIGATION_ANGLE_MAX;
        }
        process_velocity = VEL_ZERO + conf.velocityBack + wrongway.backv;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstro, "wrongway_process", "msg=\"wrongway in progress (BACK)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2) 
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME3_STOP) {
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstro, "wrongway_process", "msg=\"wrongway in progress (STOP2)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) 
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity);
    } else {
        wrongway.ok = 1;
        wrongway.time = -1;
        LOGM_INFO(loggerIstro, "wrongway_process", "msg=\"wrongway finished!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) 
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity);
        return 0;
    }
    
    return 1;
}

const int DETECTOBST_CALIB_TIMEOUT     =  10;  /* we need to see obstacle for at least 10ms (2 frames) before telling to process_thread */
const int DETECTOBST_NOCALIB_TIMEOUT   = 110;  /* during calibration - 3 frames (2*50ms + 10ms) */

const int DETECTOBST_ANGLE_MIN =  40;
const int DETECTOBST_ANGLE_MAX = 140;

typedef struct {
    int    noobstacle;     // 1 = no obstacle, 0 = obstacle detected!
    double time;
} detectobst_t;

void detectobst_init(detectobst_t &detectobst)
{
    detectobst.noobstacle = -1;
    detectobst.time = -1;
}

int detectobst_process(detectobst_t &detectobst, int calib_ok, int process_angle_min, int process_angle_max)
{
    // xxxxxxx if process_angle_min/max is not initialized (<0) we will not report obstacle 
    int noobst = (process_angle_min <= DETECTOBST_ANGLE_MIN) && (process_angle_max >= DETECTOBST_ANGLE_MAX) &&
                 (process_angle_min >= 0) && (process_angle_max >= 0);

    if (noobst) {
        // no obstacle
        if ((detectobst.noobstacle > 0) && (detectobst.time >= 0)) {
            LOGM_TRACE(loggerIstro, "detectobst_process", "msg=\"obstacle ignored!\", noobstacle=" << detectobst.noobstacle 
                << ", dt=" << ioff(timeDelta(detectobst.time), 2) << ", process_angle_min=" << process_angle_min << ", process_angle_max=" << process_angle_max);
        }
        detectobst.noobstacle = 1;
        detectobst.time = -1;
    } else {
        // obstacle detected - wait before setting noobstacle to 0
        if (detectobst.time < 0) {
            detectobst.noobstacle = 1;
            detectobst.time = timeBegin();
        }
        if (calib_ok) {
            if (timeDelta(detectobst.time) >= DETECTOBST_CALIB_TIMEOUT) {
                detectobst.noobstacle = 0;
            }
        } else {
            if (timeDelta(detectobst.time) >= DETECTOBST_NOCALIB_TIMEOUT) {
                detectobst.noobstacle = 0;
            }
        }
    }
    
    if (detectobst.noobstacle <= 0) {
        LOGM_TRACE(loggerIstro, "detectobst_process", "msg=\"obstacle detected!\", noobstacle=" << detectobst.noobstacle 
            << ", dt=" << ioff(timeDelta(detectobst.time), 2) << ", process_angle_min=" << process_angle_min << ", process_angle_max=" << process_angle_max);
    }
    
    return detectobst.noobstacle > 0;
}

const int LOADAREA_TIME_BALL_WAIT   = 4000;    /* cakanie pred vylozenim lopticky */
const int LOADAREA_TIME_BALL_DROP   = 8000;    /* cakanie po vylozeni lopticky */

const int LOADAREA_STATE_NONE       =    0;
const int LOADAREA_STATE_BALL_WAIT  =    1;
const int LOADAREA_STATE_BALL_DROP  =    2;

typedef struct {
    int    state;     // 0 = no state, 1 = BALLUNLOAD
    double time;
    int    cnt;       // kolka lopticka ma byt vylozena, budeme to kodovat ako (PROCESS_STATE_BALLUNLOAD + cnt)
    int    navp_idx;  // v ktorom navigacnom bode sme naposledy robili "ball drop"
} loadarea_t;

void loadarea_init(loadarea_t &loadarea)
{
    loadarea.state = LOADAREA_STATE_NONE;
    loadarea.time = -1;
    loadarea.cnt = 0;
    loadarea.navp_idx = -1;
}

int loadarea_process(loadarea_t &loadarea, int gps_navp_loadarea, int ctrlb_loadd, int &process_angle, int &process_velocity, int &process_state,
        int gps_navp_idx, double gps_navp_azimuth, double qrscan_latitude, double qrscan_longitude)
{
    // wait for loading/unloading? 
    if ((gps_navp_loadarea == NAVIGATION_AREA_LOADING) || (gps_navp_loadarea == NAVIGATION_AREA_UNLOADING)) {

        if ((gps_navp_loadarea == NAVIGATION_AREA_LOADING) && (ctrlb_loadd != 1)) {
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_LOADING;
            LOGM_TRACE(loggerIstro, "loadarea_process", "msg=\"loading area - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state 
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        } else
        if ((gps_navp_loadarea == NAVIGATION_AREA_UNLOADING) && (ctrlb_loadd != 0)) {
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_UNLOADING;
            LOGM_TRACE(loggerIstro, "loadarea_process", "msg=\"unloading area - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state 
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        }
    }

    // sme v novom navigacnom bode, ktory ma priznak BALLDROP?
    if ((gps_navp_loadarea == NAVIGATION_AREA_BALLDROP) && (loadarea.navp_idx != gps_navp_idx)) {
        std::string state_str = "ball drop";
        if (loadarea.state == LOADAREA_STATE_BALL_WAIT) {
            state_str = "ball drop - waiting!";
            if (timeDelta(loadarea.time) >= LOADAREA_TIME_BALL_WAIT) {
                loadarea.state = LOADAREA_STATE_BALL_DROP;
                loadarea.cnt++;    // treba vylozit dalsiu lopticku
            }
        } else
        if (loadarea.state == LOADAREA_STATE_BALL_DROP) {
            state_str = "ball drop - dropping!";
            if (timeDelta(loadarea.time) >= LOADAREA_TIME_BALL_DROP) {            
                state_str = "ball drop - finished!";
                loadarea.state = LOADAREA_STATE_NONE;
                loadarea.navp_idx = gps_navp_idx;
            }
        } else {  
            /* LOADAREA_STATE_NONE */ 
            state_str = "ball drop - init!";
            loadarea.state = LOADAREA_STATE_BALL_WAIT;
            loadarea.time = timeBegin();
        }
        
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_BALLDROP + loadarea.cnt;
        LOGM_TRACE(loggerIstro, "loadarea_process", "msg=\"" << state_str << "\", gps_navp_loadarea=" << gps_navp_loadarea
            << ", process_state=" << process_state << ", gps_navp_idx=" << gps_navp_idx
            << ", loadarea_state=" << loadarea.state << ", loadarea_dt=" << ioff(timeDelta(loadarea.time), 2)
            << ", loadarea_cnt=" << loadarea.cnt << ", loadarea_navp_idx=" << loadarea.navp_idx);
        return 1;        
    }

    // wait for scanning qr-coordinates? 
    if (gps_navp_idx >= 0) {
        if (gps_navp_azimuth >= ANGLE_OK) {
            vision_thread_qrscan_enable();
            if ((qrscan_latitude < ANGLE_OK) && (qrscan_longitude < ANGLE_OK)) {
                navigation_point_set(gps_navp_idx, qrscan_latitude, qrscan_longitude);
            }
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_QRSCAN_COORD;
            LOGM_TRACE(loggerIstro, "loadarea_process", "msg=\"qrscan coordinates - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state 
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        } else {
            vision_thread_qrscan_disable();
            return 0;
        }
    }
    
    return 0;
}

const double UPDATEXY_DT_MIN =    0.0;
const double UPDATEXY_DT_MAX =  500.0;     /* max time interval to calculate (in miliseconds) */
const double UPDATEXY_DT_STEP =   5.0;     /* integration step (in miliseconds) */

const double UPDATEXY_IRCV500_MAX = 100.0;   /* corresponds to maximum speed of 3 metres/sec */

const double UPDATEXY_ZERO_EPS   = 0.000001;
const double UPDATEXY_SPEED_COEF = 0.03;   /* (in metres)  one impulse from encoders = 3 centimetres */
const double UPDATEXY_D_COEF     = 0.335;  /* distance from the front wheel to the rear axle (in metres) */

const double UPDATEXY_RANGLE_MAX =  90.0;   /* what relative ctrlb_velocity (difference to sa_straight) */
const double UPDATEXY_ALFA_MAX   =  18.5;   /* corresponds to what alfa*/

int update_xy(double &process_x, double &process_y, double &process_yaw, double &process_time,
        double ctrlb_ircv500, int ctrlb_angle, int ctrlb_velocity)
{
    double t = timeBegin();
    double dt = timeDelta2(process_time, t);

    process_time = t;
    if ((process_yaw >= ANGLE_OK) || (ctrlb_angle < 0) || (ctrlb_angle < 0) || (ctrlb_ircv500 < 0) || (ctrlb_ircv500 > UPDATEXY_IRCV500_MAX)) {
        LOGM_ERROR(loggerIstro, "update_xy", "msg=\"error: ctrlb values not initialised!\"");
        return -1;
    }
    if ((dt < UPDATEXY_DT_MIN) || (dt > UPDATEXY_DT_MAX)) {
        LOGM_ERROR(loggerIstro, "update_xy", "msg=\"error: max time exceeded!\", dt=" << ioff(dt, 2));
        return -2;
    }
    
    /* save old values */
    double process_x_old = process_x;
    double process_y_old = process_y;
    double process_yaw_old = process_yaw;
    double dt_old = dt;
    
    /* calculate robot speed from encoders */
    double speed = ctrlb_ircv500 * UPDATEXY_SPEED_COEF;
    if (ctrlb_velocity < VEL_ZERO) {
        speed = -speed;    // we are going backward
    }
    
    /* calculate relative angle of the front wheel from ctrlb_angle */
    /* see test "170817_test_SadJK\out_1803_maxangle" */
    /* maximum angle 455 coresponds to curvature radius 1 meter => which coresponds to alfa = 18,5 degrees */
    double da = (ctrlb_angle - SA_STRAIGHT);
    if (da > UPDATEXY_RANGLE_MAX) {
        da = UPDATEXY_RANGLE_MAX;
    }
    if (da < -UPDATEXY_RANGLE_MAX) {
        da = -UPDATEXY_RANGLE_MAX;
    }
    /* orientation is ok (ctrlb_angle=245 means clockwise => -18,5 degrees)*/
    double alfa = UPDATEXY_ALFA_MAX * da / UPDATEXY_RANGLE_MAX;
    
    /* perform integration over time */
    while (dt > UPDATEXY_ZERO_EPS) {
        double dt0 = dt;
        if (dt0 > UPDATEXY_DT_STEP) {
            dt0 = UPDATEXY_DT_STEP;
        }
        /* see odometry2.png - yaw = theta */
        process_x += (dt0 / 1000.0) * speed * cos(alfa * M_PI / 180.0) * sin(process_yaw * M_PI / 180.0);
        process_y += (dt0 / 1000.0) * speed * cos(alfa * M_PI / 180.0) * cos(process_yaw * M_PI / 180.0);
        process_yaw += ((dt0 / 1000.0) * speed * sin(alfa * M_PI / 180.0) / UPDATEXY_D_COEF) * (180.0 / M_PI);
        dt -= dt0;
    }

    LOGM_DEBUG(loggerIstro, "update_xy", 
        "process_x=" << ioff(process_x, 3) << ", process_y=" << ioff(process_y, 3)
        << ", process_yaw=" << ioff(process_yaw, 3) << ", ctrlb_ircv500=" << ioff(ctrlb_ircv500, 2) 
        << ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity
        << ", speed=" << ioff(speed, 2) << ", alfa=" << ioff(alfa, 2) << ", dt=" << ioff(dt_old, 2)
        << ", process_x_old=" << ioff(process_x_old, 3) << ", process_y_old=" << ioff(process_y_old, 3)
        << ", process_yaw_old=" << ioff(process_yaw_old, 3));
    
    timeEnd("istro::update_xy", t);
    
    return 0;
}

void update_xy_test(void)
{
    double process_x = 0.0; 
    double process_y = 0.0;
    double process_yaw = 0.0;
    double process_time = timeBegin();
    double ctrlb_ircv500 = 15.0;
    int ctrlb_angle = SA_MIN; 
    int ctrlb_velocity = VEL_OPT;

    process_time = timeBegin();
    for(int i = 0; i < 10*100; i++) {
        update_xy(process_x, process_y, process_yaw, process_time, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity);
        msleep(10);
    }
}

const int SPEEDCTL_MINMAX_CNT =     15;  // pri menej ako 15 preruseniach (cca 300ms) po sebe od MINMAX neresetuj timer pre SPEEDCTL_V2/V3_TIME

const int SPEEDCTL_V3_DIST =       300;  // (cm) v akej vzdialenosti analyzujeme
const int SPEEDCTL_V3_INT_LENGTH =  40;  // aky siroky interval hladame, podobne ako DMAP_MIN_INTERVAL_LENGTH
const int SPEEDCTL_V3_SHIFT_MAX =  -10;  // kolko moze byt maximalny odklon od stredu, aby mohol ist robot rychlostou V3
const int SPEEDCTL_V3_TIME =      4000;  // po akom case sa moze zvysit rychlost na V3

const int SPEEDCTL_V2_DIST =       200;  // (cm) v akej vzdialenosti analyzujeme
const int SPEEDCTL_V2_INT_LENGTH =  30;  // aky siroky interval hladame, podobne ako DMAP_MIN_INTERVAL_LENGTH
const int SPEEDCTL_V2_SHIFT_MAX =    0;  // kolko moze byt maximalny odklon od stredu, aby mohol ist robot rychlostou V3
const int SPEEDCTL_V2_TIME =      2000;  // po akom case sa moze zvysit rychlost na V2

typedef struct {
    int minmax_cnt;    // kolko krat bol minmax
    int velocity_last;
    int state_last;
    double time;
    double time2;
    double time3;
} speedctl_t;

void speedctl_init(speedctl_t &speedctl)
{
    speedctl.minmax_cnt = 0;
    speedctl.time = -1;
    speedctl.time2 = -1;
    speedctl.time3 = -1;
    speedctl.velocity_last = -1;
    speedctl.state_last = -1;
}

void speedctl_timeReset(speedctl_t &speedctl)
{
    speedctl.minmax_cnt = 0;
    speedctl.time = -1;	
}

int speedctl_getshift(int dir, int angle1, int angle2)
// vrati vzdialenost prekazky od planovaneho smeru pohybu robota - cim vacsia, tym pomalsie sa treba pohybovat
// angle1, angle2 su hranice intervalu, ktory vratil dmap_find() pre odporucany pohyb robota
// vysledok je vzdialenost od pozadovaneho smeru, kladna znamena o kolko sa musi vychylit kraj
//   zaporna hodnota znamena, z
{
    if ((angle1 >= 0) && (angle2 >= 0)) {
        int sh1 = angle1 - dir;
       	if (sh1 < 0) sh1 = -sh1;
        int sh2 = angle2 - dir;
      	if (sh2 < 0) sh2 = -sh2;
        if ((angle1 <= dir) && (angle2 >= dir)) {
            return (sh1 < sh2)?-sh1:-sh2;
        }
        return (sh1 < sh2)?sh1:sh2;
    }
    return 180;
}

int speedctl_process(speedctl_t &speedctl, const DegreeMap &dmap, int process_dir, int &process_velocity, int &process_state, int &process_stop)
{
    /* change speed only in NAV_ANGLE and MIN_MAX states */
	if ((process_state != PROCESS_STATE_NAV_ANGLE) && 
        (process_state != PROCESS_STATE_MIN_MAX) &&
        (process_state != PROCESS_STATE_CALIBRATION)) {
        speedctl_timeReset(speedctl);
        speedctl.state_last = process_state;
        return 0;
    }
    if ((process_stop) || (process_velocity >= 0)) {
        speedctl_timeReset(speedctl);
        speedctl.state_last = process_state;
	return 0;
    }

    int velocity = -1;
    int velocity_old = process_velocity;

    if (process_state == PROCESS_STATE_CALIBRATION) {
    	velocity = VEL_ZERO + conf.velocityFwd;
 
        speedctl_timeReset(speedctl);
        LOGM_DEBUG(loggerIstro, "speedctl_process", 
            "velocity=" << velocity << ", process_state=" << process_state 
            << ", process_dir=" << process_dir
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    } else 
    if (process_state == PROCESS_STATE_MIN_MAX) {
    	velocity = VEL_ZERO + conf.velocityFwd;

    	// ak bol predchadzajuci stav NAV_ANGLE, tak zvysuj minmax_cnt a resetni NAV_ANGLE casy iba po prekroceni MINMAX_CNT
    	if (speedctl.time >= 0) {
    		speedctl.minmax_cnt++;
    		if (speedctl.minmax_cnt > SPEEDCTL_MINMAX_CNT) {
                speedctl_timeReset(speedctl);
    		}
    	}
        LOGM_DEBUG(loggerIstro, "speedctl_process", 
            "velocity=" << velocity << ", process_state=" << process_state 
            << ", process_dir=" << process_dir << ", minmax_cnt=" << speedctl.minmax_cnt
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    } else 
    if (process_state == PROCESS_STATE_NAV_ANGLE) {
    	speedctl.minmax_cnt = 0;
        if (speedctl.time < 0) {
        	speedctl.time = speedctl.time2 = speedctl.time3 = timeBegin();
        }           

        int angle_min1, angle_max1, sh1;
        int angle_min2, angle_max2, sh2;
        int angle_min3, angle_max3, sh3;

    	DegreeMap dmap2;
    	dmap2.init();
    	dmap2.copy(dmap);

        //dmap2.print("speedctl_process::dmap2.v1");
        dmap2.find(DMAP_MIN_INTERVAL_LENGTH, process_dir, angle_min1, angle_max1);
        sh1 = speedctl_getshift(process_dir, angle_min1, angle_max1);
        dmap2.remap(SPEEDCTL_V2_DIST);
        //dmap2.print("speedctl_process::dmap2.v2");
        dmap2.find(SPEEDCTL_V2_INT_LENGTH, process_dir, angle_min2, angle_max2);
        sh2 = speedctl_getshift(process_dir, angle_min2, angle_max2);
    	dmap2.remap(SPEEDCTL_V3_DIST);
        //dmap2.print("speedctl_process::dmap2.v3");
    	dmap2.find(SPEEDCTL_V3_INT_LENGTH, process_dir, angle_min3, angle_max3);
        sh3 = speedctl_getshift(process_dir, angle_min3, angle_max3);

        int sf = 0;
        if (sh2 > SPEEDCTL_V2_SHIFT_MAX) {
            sf += 1;
            speedctl.time2 = timeBegin();
        }
        if (sh3 > SPEEDCTL_V3_SHIFT_MAX) {
            sf += 2;
        	speedctl.time3 = timeBegin();
        } 

        velocity = VEL_ZERO + conf.velocityFwd;
        if (timeDelta(speedctl.time3) >= SPEEDCTL_V3_TIME) {
            sf += 8;
            velocity = VEL_ZERO + conf.velocityFwd3;
        } else
        if (timeDelta(speedctl.time2) >= SPEEDCTL_V2_TIME) {
            sf += 4;
            velocity = VEL_ZERO + conf.velocityFwd2;
        }

        LOGM_DEBUG(loggerIstro, "speedctl_process", 
            "velocity=" << velocity << ", process_state=" << process_state 
            << ", process_dir=" << process_dir
            << ", sf=" << sf << ", sh1=" << sh1 << ", sh2=" << sh2 << ", sh3=" << sh3
            << ", angle_min1=" << angle_min1 << ", angle_max1=" << angle_max1 
            << ", angle_min2=" << angle_min2 << ", angle_max2=" << angle_max2 
            << ", angle_min3=" << angle_min3 << ", angle_max3=" << angle_max3
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    }

    if (velocity > 0) {
        if ((speedctl.state_last != process_state) || (speedctl.velocity_last != velocity)) {
            process_velocity = velocity;
            LOGM_DEBUG(loggerIstro, "speedctl_process", "msg=\"set process_velocity!\"" 
                << "velocity=" << velocity);
        }
        speedctl.velocity_last = velocity;
    }

    speedctl.state_last = process_state;
    return 1;
}

int process_readData(double &gps_time, double &gps_speed, double &gps_course, /* double &gps_latitude, double &gps_longitude, */
        double &gps_navp_dist, double &gps_navp_azimuth, double &gps_navp_maxdist, int &gps_navp_loadarea, int &gps_navp_idx,
        double &ctrlb_time1, double &ctrlb_ircv500, int &ctrlb_angle, int &ctrlb_velocity, int &ctrlb_loadd,
        double &ctrlb_time2, double &ctrlb_euler_x, double &ahrs_yaw, 
        double &gps_lastp_azimuth, int &gps_ref, double &gps_x, double &gps_y) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    gps_time = pdata->gps_time;
    gps_speed = pdata->gps_speed;
    gps_course = pdata->gps_course;
/*  gps_latitude = pdata->gps_latitude;  */
/*  gps_longitude = pdata->gps_longitude;  */
    gps_navp_dist = pdata->gps_navp_dist;
    gps_navp_azimuth = pdata->gps_navp_azimuth;
    gps_navp_maxdist = pdata->gps_navp_maxdist;
    gps_navp_loadarea = pdata->gps_navp_loadarea;
    gps_navp_idx = pdata->gps_navp_idx;
    
    gps_lastp_azimuth = pdata->gps_lastp_azimuth;
    gps_ref = pdata->gps_ref;
    gps_x = pdata->gps_x;
    gps_y = pdata->gps_y;

    ctrlb_time1 = pdata->ctrlb_time1;
    ctrlb_ircv500 = pdata->ctrlb_ircv500;
    ctrlb_angle = pdata->ctrlb_angle;
    ctrlb_velocity = pdata->ctrlb_velocity;
    ctrlb_loadd = pdata->ctrlb_loadd;
    ctrlb_time2 = pdata->ctrlb_time2; 
    ctrlb_euler_x = pdata->ctrlb_euler_x;

    ahrs_yaw = pdata->ahrs_yaw;
    
/*    // logging is performed only if timestamps will change
    LOGM_DEBUG(loggerIstro, "process_readData", "fix=" << pdata->gps_fix << ", latitude=" << ioff(pdata->gps_latitude, 6) << ", longitude=" << ioff(pdata->gps_longitude, 6) << ", course=" << ioff(pdata->gps_course, 2) << 
        ", lastp_dist=" << ioff(pdata->gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(gps_lastp_azimuth, 2) << 
        ", ctrlb_ircv500=" << ioff(pdata->ctrlb_ircv500, 2) <<
        ", ctrlb_angle=" << pdata->ctrlb_angle << ", ctrlb_velocity=" << pdata->ctrlb_velocity << ", ctrlb_loadd=" << pdata->ctrlb_loadd <<
        ", euler_x=" << ioff(pdata->ctrlb_euler_x, 2) << ", ahrs_yaw=" << ioff(pdata->ahrs_yaw, 2) << 
        ", navp_dist=" << ioff(pdata->gps_navp_dist, 3) << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", navp_maxdist=" << ioff(pdata->gps_navp_maxdist, 3) <<
        ", navp_loadarea=" << pdata->gps_navp_loadarea << ", navp_idx=" << navp_idx <<
        ", gps_ref=" << pdata->gps_ref << ", gps_x=" << ioff(pdata->gps_x, 2) << ", gps_y=" << ioff(pdata->gps_y, 2));
*/    
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::process_readData", t);
#endif

    return 0;
}

int process_writeData(long image_number, int process_angle, int process_velocity, int process_state, int process_stop,
        int process_dir, int process_ref, double process_x, double process_y, double process_yaw, int coneseek_stop, int coneseek_intlen)
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->image_number = image_number;
    pdata->process_angle = process_angle;
    pdata->process_velocity = process_velocity;
    pdata->process_state = process_state;
    pdata->process_stop = process_stop;

    pdata->process_dir = process_dir;
    pdata->process_ref = process_ref;
    pdata->process_x = process_x;
    pdata->process_y = process_y;
    pdata->process_yaw = process_yaw;

    pdata->coneseek_stop = coneseek_stop;
    pdata->coneseek_intlen = coneseek_intlen;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
    
    LOGM_DEBUG(loggerIstro, "process_writeData", "image_number=" << image_number
        << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity 
        << ", process_state=" << process_state << ", process_stop=" << process_stop
        << ", process_dir=" << process_dir
        << ", process_ref=" << process_ref << ", process_x=" << ioff(process_x, 2) << ", process_y=" << ioff(process_y, 2)
        << ", process_yaw=" << ioff(process_yaw, 2) << ", coneseek_stop=" << coneseek_stop);

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::process_writeData", t);
#endif

    return 0;
}

const double PROCESS_DIR_MIN =  75;   // 45-135 caused a lot of errors, trying to go directly to grass
const double PROCESS_DIR_MAX = 105;

const int    PROCESS_DIST_MAX = 300;  // just for dmap init

const int    PROCESS_PERIOD_MIN = 20;  // how often will we process new data from sensors (goal is every 20ms to have new decision)
const int    PROCESS_LAG_PERIOD = 3*20;  // if no data are comming from control board (euler_x), report processing lag

const int    PROCESS_NOANGLE_TO = 1200;
const int    PROCESS_NOANGLE_MIN =  30;  // is it possible with wmgrid??
const int    PROCESS_NOANGLE_MAX = 150;

const int    PROCESS_CHANGE_GPS = 8;
const int    PROCESS_CHANGE_CTRLB1 = 16;
const int    PROCESS_CHANGE_CTRLB2 = 32;

const double PROCESS_LIDAR_SHRINK = 0.60;  // shrink factor for lidar data - process_dmap detects obstacles below 1m, we have to draw them closer into grid [150 -> 90]

int angle_fixi(int angle)
{
    while (angle > 180) {
        angle -= 360; 
    }
    while (angle <= -180) {
        angle += 360;
    }
    return angle;
}

double angle_fixd(double angle)
{
    while (angle > 180.0) {
        angle -= 360.0; 
    }
    while (angle <= -180.0) {
        angle += 360.0;
    }
    return angle;
}

void process_thread_test(void) 
{
    //qrscan.test();
    //wmodel.test();
    //update_xy_test();
    //navmap_test();
    
    //for(int i = 0; i < 5; i++) {
    //    double t = timeBegin();
    //    msleep(2);
    //    timeEnd("istro::process_thread:test_2ms", t);
    //}
}

const int CONESEEK_STOP_INTLEN = 13;    /* pri akej velkosti kuzela (v stupnoch) zastavit a vylozit lopticku */

void *process_thread(void *parg)
{
    double t = -1;
    double tw = -1;
    double last_gps_time = -1;
    double last_ctrlb_time1 = -1;
    double last_ctrlb_time2 = -1;
    
    int last_process_angle_min = -1;
    int last_process_angle_max = -1;

    int result = 0;
    long image_number = 0;
    double process_to;
    double minmax_lastt;  // when was the last process_angle calculated with MIN_MAX method?
    double noangle_to;
    
    calib_t calib;
    calib_init(calib);
    
    wrongway_t wrongway;
    wrongway_init(wrongway);
    
    detectobst_t detectobst;
    detectobst_init(detectobst);

    speedctl_t speedctl;
    speedctl_init(speedctl);

    loadarea_t loadarea;
    loadarea_init(loadarea);

    DataSet data;
    DataSet *pdata;

    LOG_THREAD_NAME("process");
    LOGM_INFO(loggerIstro, "process_thread", "msg=\"start\"");

    try {
    process_thread_test();

    int process_change = 0;
    process_to = timeBegin();
    minmax_lastt = timeBegin();
    noangle_to = timeBegin();
    while (!thread_testcancel() && (conf.useCamera || conf.useLidar)) {
        int do_process = 0;
        if (tw < 0) {
            tw = timeBegin();
        }

        /* update wmodel based on VISION data */
        if (conf.useCamera) {
            pdata = (DataSet *)threads.checkData(THDATA_STATE_VISION_PROCESSED, THDATA_STATE_PROCESSING);
            if (pdata != NULL) {
                double t2 = timeBegin();
#ifdef ISTRO_VISION_ORANGECONE
                /* do not write cone positions to obstacle grid, just calculate coneseek values */
                LOGM_DEBUG(loggerIstro, "process_thread", "msg=\"process_vision_coneseek\"");
                pdata->vision_dmap.findbnd(pdata->coneseek_angle_min, pdata->coneseek_angle_max, pdata->coneseek_intlen);
                pdata->coneseek_stop = (pdata->coneseek_intlen >= CONESEEK_STOP_INTLEN);
                LOGM_INFO(loggerIstro, "process_thread", "dmap_vision: image_number=" << image_number << ", coneseek_angle_min=" << pdata->coneseek_angle_min << ", coneseek_angle_max=" << pdata->coneseek_angle_max 
                    << ", coneseek_intlen=" << pdata->coneseek_intlen << ", coneseek_stop=" << pdata->coneseek_stop << ", process_dir=" << pdata->process_dir);
                //pdata->vision_dmap.print("vision_dmap");
                // copy "coneseek output" from vision dataset to local-dataset
                data.coneseek_angle_min = pdata->coneseek_angle_min;
                data.coneseek_angle_max = pdata->coneseek_angle_max;
                data.coneseek_intlen = pdata->coneseek_intlen;
                data.coneseek_stop = pdata->coneseek_stop;
#else
                LOGM_DEBUG(loggerIstro, "process_thread", "msg=\"process_vision\"");
                if ((pdata->process_ref == data.process_ref) && (pdata->process_yaw < ANGLE_OK)) {
                    wmodel.updateGrid(pdata->vision_dmap, pdata->process_x, pdata->process_y, pdata->process_yaw, 
                        data.process_ref, data.process_angle, data.process_angle_min, data.process_angle_max,
                        WMGRID_VISION_MBIT, WMGRID_VISION_VBIT, image_number);
                }
#endif
                if (pdata->lidar_data_cnt > 0) {
                    pdata->lidar_data_cnt = -1;  // clear lidar_data to inform save_thread, that only camera data should be written to disk
                }
                // copy latest local variables to dataset prepared for writing (for logging purposes in save_thread)
                pdata->process_angle = data.process_angle;
                pdata->process_angle_min = data.process_angle_min;
                pdata->process_angle_max = data.process_angle_max;
                // copy "qrscan output" from vision dataset to local-dataset
                data.qrscan_latitude = pdata->qrscan_latitude;
                data.qrscan_longitude = pdata->qrscan_longitude;
                threads.setData(pdata, THDATA_STATE_PROCESSED, 2);
                //do_process = 1;
                process_change += 1;
#ifdef ISTRO_VISION_ORANGECONE
                timeEnd("istro::process_thread.process_vision_coneseek", t2);
#else
                timeEnd("istro::process_thread.process_vision", t2);
#endif
            }
        }

        /* update wmodel based on LIDAR data */
        if (conf.useLidar) {
            pdata = (DataSet *)threads.checkData(THDATA_STATE_LIDAR_CAPTURED, THDATA_STATE_PROCESSING);                
            if (pdata != NULL) {
                double t2 = timeBegin();
                LOGM_DEBUG(loggerIstro, "process_thread", "msg=\"process_lidar\"");
                lidar.process(pdata->lidar_data, pdata->lidar_data_cnt, pdata->lidar_dmap, pdata->lidar_stop);
                pdata->process_dmap.apply(pdata->lidar_dmap);  // fixme: process_dmap is never cleared/initialised in lidar_thread! not used at all?
                //pdata->lidar_dmap.print("lidar_dmap");
                
                data.process_stop = pdata->lidar_stop;    // copy lidar variable to local dataset                

                pdata->lidar_dmap.find(DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->lidar_angle_min, pdata->lidar_angle_max);
                LOGM_INFO(loggerIstro, "process_thread", "dmap_lidar: lidar_angle_min=" << pdata->lidar_angle_min << ", lidar_angle_max=" << pdata->lidar_angle_max 
                    << ", lidar_stop=" << pdata->lidar_stop << ", process_dir=" << pdata->process_dir);

                // update grid only if the reference was the same (dont put uncalibrated data to already calibrated grid)
                if ((pdata->process_ref == data.process_ref) && (pdata->process_yaw < ANGLE_OK)) {
                    pdata->lidar_dmap.shrink(PROCESS_LIDAR_SHRINK);
                    wmodel.updateGrid(pdata->lidar_dmap, pdata->process_x, pdata->process_y, pdata->process_yaw, 
                        data.process_ref, data.process_angle, data.process_angle_min, data.process_angle_max,
                        WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
                }
                if (!pdata->camera_img.empty()) {
                    pdata->camera_img.release();  // clear camera_img to inform save_thread, that only lidar data should be written to disk
                }
                // lidar image shows process_angle information - we need to copy latest local variables to dataset prepared for writing
                pdata->process_angle = data.process_angle;
                pdata->process_angle_min = data.process_angle_min;
                pdata->process_angle_max = data.process_angle_max;
                threads.setData(pdata, THDATA_STATE_PROCESSED, 2);
                //do_process = 1;
                process_change += 2;
                timeEnd("istro::process_thread.process_lidar", t2);
            }
        }

        // for processing we use local dataset
        pdata = &data;

        // every 20ms we should get new data from control_board -> this event will trigger the processing
        if (timeDelta(process_to) >= PROCESS_PERIOD_MIN) {
            /* read data from GPS and control board (shared dataset)*/
            if (process_readData(pdata->gps_time, pdata->gps_speed, pdata->gps_course, /*pdata->gps_latitude, pdata->gps_longitude,*/
                                 pdata->gps_navp_dist, pdata->gps_navp_azimuth, pdata->gps_navp_maxdist, pdata->gps_navp_loadarea, pdata->gps_navp_idx,
                                 pdata->ctrlb_time1, pdata->ctrlb_ircv500, pdata->ctrlb_angle, pdata->ctrlb_velocity, pdata->ctrlb_loadd, 
                                 pdata->ctrlb_time2, pdata->ctrlb_euler_x, pdata->ahrs_yaw, 
                                 pdata->gps_lastp_azimuth, pdata->gps_ref, pdata->gps_x, pdata->gps_y) < 0) {
                result = -2;
                break;
            }

            // dont process data if ctrlb_euler_x is not known!! 
            if (pdata->ctrlb_euler_x < ANGLE_OK) {
            if ((last_gps_time != pdata->gps_time) || (last_ctrlb_time1 != pdata->ctrlb_time1) || (last_ctrlb_time2 != pdata->ctrlb_time2)) {
                // fixme: AHRS timestamp is not supported
                do_process = 1;
                process_change += 4;
                if (last_gps_time != pdata->gps_time) {
                    process_change += PROCESS_CHANGE_GPS;
                }
                if (last_ctrlb_time1 != pdata->ctrlb_time1) {
                    process_change += PROCESS_CHANGE_CTRLB1;
                }
                if (last_ctrlb_time2 != pdata->ctrlb_time2) {
                    process_change += PROCESS_CHANGE_CTRLB2;
                }
                LOGM_DEBUG(loggerIstro, "process_readData", // "fix=" << pdata->gps_fix << ", latitude=" << ioff(pdata->gps_latitude, 6) << ", longitude=" << ioff(pdata->gps_longitude, 6) << 
                    "process_change=" << process_change << ", gps_speed=" << ioff(pdata->gps_speed, 3) << ", gps_course=" << ioff(pdata->gps_course, 2) << 
                    ", lastp_dist=" << ioff(pdata->gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(pdata->gps_lastp_azimuth, 2) << 
                    ", ctrlb_ircv500=" << ioff(pdata->ctrlb_ircv500, 2) <<
                    ", ctrlb_angle=" << pdata->ctrlb_angle << ", ctrlb_velocity=" << pdata->ctrlb_velocity << ", ctrlb_loadd=" << pdata->ctrlb_loadd <<
                    ", euler_x=" << ioff(pdata->ctrlb_euler_x, 2) << ", ahrs_yaw=" << ioff(pdata->ahrs_yaw, 2) << 
                    ", navp_dist=" << ioff(pdata->gps_navp_dist, 3) << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", navp_maxdist=" << ioff(pdata->gps_navp_maxdist, 3) <<
                    ", navp_loadarea=" << pdata->gps_navp_loadarea << ", navp_idx=" << pdata->gps_navp_idx <<
                    ", gps_ref=" << pdata->gps_ref << ", gps_x=" << ioff(pdata->gps_x, 2) << ", gps_y=" << ioff(pdata->gps_y, 2));
                last_gps_time = pdata->gps_time;
                last_ctrlb_time1 = pdata->ctrlb_time1;
                last_ctrlb_time2 = pdata->ctrlb_time2;
            }
            }
        }
        
        if (!do_process) {
            // no new data -> busy wait 
            msleep(2);
            continue;
        }
        timeEnd("istro::process_thread.wait", tw);
        tw = -1;
        t = timeBegin();
        
        /* detect processing lag */
        int process_dt = timeDelta(process_to);
        if (process_dt > PROCESS_LAG_PERIOD) {
            LOGM_DEBUG(loggerIstro, "process_thread", "msg=\"processing_lag detected!\", image_number=" << image_number << ", process_dt=" << process_dt);
            /* processing_lag should not cause wrongway */
            if (process_dt > (int)(PROCESS_NOANGLE_TO / 3) ) {
                noangle_to = timeBegin();
            }
        }
        process_to = timeBegin();

        /* processing start */
        LOGM_DEBUG(loggerIstro, "process_thread", "msg=\"do_process\", process_dt=" << process_dt <<
            ", process_change1=" << (process_change & 7) << ", process_change2=" << ((process_change >> 3) & 7));
            
        /* update "process_ref" */
        int process_ref_old = pdata->process_ref;
        if (!pdata->process_ref && (pdata->gps_ref > 0) && (calib.azimuth < ANGLE_OK) && (calib.yaw < ANGLE_OK)) {
            /* set to 1 if the gps position and calibration values are known */
            pdata->process_ref = 1;
            // clear grid - because the coordinate system will be changed
            wmodel.init();
            LOGM_INFO(loggerIstro, "process_thread", "msg=\"process_ref set...\", process_ref=" << pdata->process_ref
                  << ", calib.azimuth=" << ioff(calib.azimuth, 2) << ", calib.yaw=" << ioff(calib.yaw, 2));
        }        

        /* calculate "yaw" */
        int yaw_src;
        double yaws = ANGLE_NONE;  // raw yaw value from sensor - 0 is not north
        double yawc = ANGLE_NONE;  // calibrated yaw value (azimuth) - 0 means north
        /* use "process_yaw" - take last yaw value that was calculated by update_xy() procedure */
        yaw_src = 0;
        if (pdata->process_ref) {
            if (process_ref_old) {
                yaws = angle_fixd(pdata->process_yaw - calib.azimuth + calib.yaw);
                yawc = pdata->process_yaw;
            } else {
                yaws = pdata->process_yaw;
                yawc = angle_fixd(pdata->process_yaw - calib.yaw + calib.azimuth);
            }
        } else {
            yaws = pdata->process_yaw;  
            yawc = ANGLE_NONE;
        }

        /* use "ctrlb_euler_x" - overwrite calculated yaw with new euler_x value from compass (only if new value was received) */
        /* assert: ctrlb_euler_x < ANGLE_OK */
        if ((process_change & PROCESS_CHANGE_CTRLB2) > 0) {
            yaw_src = 1;
            if (pdata->process_ref) {
                yaws = pdata->ctrlb_euler_x;
                yawc = angle_fixd(pdata->ctrlb_euler_x - calib.yaw + calib.azimuth);
            } else {
                yaws = pdata->ctrlb_euler_x;
                yawc = ANGLE_NONE;
            }
        }
/*        
        if ((yaw >= ANGLE_OK) && (pdata->ahrs_yaw < ANGLE_OK)) {
            yaw = pdata->ahrs_yaw;
            yaw_src = 2;
        }
        if ((yaw >= ANGLE_OK) && ((process_change & PROCESS_CHANGE_GPS) > 0)) {
            yaw = pdata->gps_lastp_azimuth;
            yaw_src = 3;
        }
*/
        if (pdata->process_ref) {
            pdata->process_yaw = yawc;
        } else {
            pdata->process_yaw = yaws;
        }
        
        /* calculate "nav_angle" */
        double yawn = ANGLE_NONE;  // yawn = yaw that was used to calculate nav_angle
        double nav_angle = ANGLE_NONE;
        /* YAW */
        if ((conf.navigationImuYaw < (int)ANGLE_OK) && (yaws < ANGLE_OK)) {
            nav_angle = angle_fixd(conf.navigationImuYaw - yaws);
            yawn = yaws;
            LOGM_TRACE(loggerIstro, "process_thread", "calc_nav_angle(\"YAW\"): image_number=" << image_number << ", nav_angle=" << ioff(nav_angle, 2) 
                << ", nav_yaw=" << conf.navigationImuYaw << ", process_yaw=" << ioff(pdata->process_yaw, 2) 
                << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yaw_src=" << yaw_src);
        } else
        /* AZIMUTH_YAW */
        if ((pdata->gps_navp_azimuth < ANGLE_OK) && (yawc < ANGLE_OK)) {
            nav_angle = angle_fixd(pdata->gps_navp_azimuth - yawc);
            yawn = yawc;
            LOGM_TRACE(loggerIstro, "process_thread", "calc_nav_angle((\"AZIMUTH_YAW\"): image_number=" << image_number << ", nav_angle=" << ioff(nav_angle, 2) 
                << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", process_yaw=" << ioff(pdata->process_yaw, 2) 
                << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yaw_src=" << yaw_src
                << ", calib.yaw=" << ioff(calib.yaw, 2) << ", calib.azimuth=" << ioff(calib.azimuth, 2));
        }

        /* wrongway_check */
        if ((yawn < ANGLE_OK) && (nav_angle < ANGLE_OK)) {
            // (angle > -180) && (angle <= 180)
            wrongway_check(wrongway, yawn, nav_angle, pdata->gps_navp_dist, pdata->gps_navp_maxdist);
        }

        /* calculate "obstacle distance map" -> "process_angle_min/max" */
        pdata->process_dir = 90;
        if (nav_angle < ANGLE_OK) {
            pdata->process_dir = 90 - trunc(nav_angle);
            if (pdata->process_dir < PROCESS_DIR_MIN) {
                pdata->process_dir = PROCESS_DIR_MIN;
            }
            if (pdata->process_dir > PROCESS_DIR_MAX) {
                pdata->process_dir = PROCESS_DIR_MAX;
            }
        }
        // find obstacles in the wmgrid
        int eval_res = wmodel.evalGrid(pdata->process_x, pdata->process_y, pdata->process_yaw, pdata->process_dmap);
        //pdata->process_dmap.print("process_dmap");
        pdata->process_dmap.find(DMAP_MIN_INTERVAL_LENGTH, pdata->process_dir, pdata->process_angle_min, pdata->process_angle_max);
        LOGM_TRACE(loggerIstro, "process_thread", "msg=\"process_dmap\", image_number=" << image_number
            << ", process_ref=" << pdata->process_ref << ", process_x=" << ioff(pdata->process_x, 2) << ", process_y=" << ioff(pdata->process_y, 2)
            << ", process_yaw=" << ioff(pdata->process_yaw, 2) << ", nav_angle=" << ioff(nav_angle, 2)
            << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max);
            
        // fixme: fix uninitialized min/max if the wmodel is not consistent (because a recenter is needed) - we cant detect obstacles!!
        if (eval_res < 0) {
            LOGM_TRACE(loggerIstro, "process_thread", "msg=\"fix process_angle_min/max\", image_number=" << image_number
                << ", eval_res=" << eval_res
                << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max
                << ", last_process_angle_min=" << last_process_angle_min << ", last_process_angle_max=" << last_process_angle_max);
            pdata->process_angle_min = last_process_angle_min;
            pdata->process_angle_max = last_process_angle_max;
        } else {
            last_process_angle_min = pdata->process_angle_min;
            last_process_angle_max = pdata->process_angle_max;
        }

#ifdef WIN32
//LOGM_WARN(loggerIstro, "process_thread", "clear process_angle_min/max, DEBUG ONLY!!");
//pdata->process_angle_min = 0;
//pdata->process_angle_max = 180;
#endif

        /* calculate "process_angle" */
        pdata->process_angle = (int)ANGLE_NONE;        // fixme: process_angle na stred 0 clockwise prerobit, namiesto 90 a counterclockwise
        pdata->process_velocity = -1;
        pdata->process_state = PROCESS_STATE_NONE;

        /* LOAD/UNLOAD AREA - wait for un/loading + scanning geo-coordinates */
        loadarea_process(loadarea, pdata->gps_navp_loadarea, pdata->ctrlb_loadd, pdata->process_angle, pdata->process_velocity, pdata->process_state,
            pdata->gps_navp_idx, pdata->gps_navp_azimuth, pdata->qrscan_latitude, pdata->qrscan_longitude);

        /* DETECT OBSTACLES */
        detectobst_process(detectobst, calib.ok, pdata->process_angle_min, pdata->process_angle_max);

        /* calculate calibration */
        int calib_res = 0;
        if (detectobst.noobstacle <= 0) {
            stringstream ss;
            ss << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max;
            calib_res = calib_reset(calib, "obstacle", ss.str());
        } else 
        if (pdata->process_angle < (int)ANGLE_OK) {
            calib_res = calib_reset(calib, "loadarea", "");
        } else {
            calib_res = calib_process(calib, nav_angle, pdata->process_angle_min, pdata->process_angle_max,
                            pdata->gps_course, yaws, pdata->gps_speed, pdata->gps_navp_dist, pdata->gps_navp_maxdist, pdata->ctrlb_ircv500);
        }
        
        /* WRONG_WAY */
        if (pdata->process_angle >= (int)ANGLE_OK) {
            if (wrongway_process(wrongway, yawn, pdata->process_angle, pdata->process_velocity, pdata->process_state) > 0) {
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"WRONG_WAY\"): image_number=" << image_number << ", process_angle=" << pdata->process_angle << ", process_velocity=" << pdata->process_velocity);
            }
        } else {
            // ongoing loadarea operations must cancel wrongway calculation
            wrongway_reset(wrongway, "loadarea");
        }

        /* NO OBSTACLE - perform calibration or go to cone / navigation angle */
        if ((pdata->process_angle >= (int)ANGLE_OK) && (detectobst.noobstacle > 0)) {
            /* CALIBRATION */
            if (calib_res > 0) {
                pdata->process_angle = 90;
                pdata->process_state = PROCESS_STATE_CALIBRATION;
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"CALIBRATION\"): image_number=" << image_number << ", process_angle=" << pdata->process_angle);
            } else
            /* CONE_SEEK - cone detected? */
            if ((pdata->coneseek_angle_min >= 0) && (pdata->coneseek_angle_max >= 0)) {
                pdata->process_angle = (pdata->coneseek_angle_min + pdata->coneseek_angle_max)/2; 
                pdata->process_state = PROCESS_STATE_CONE_SEEK;
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"CONE_SEEK\"): image_number=" << image_number
                    << ", coneseek_angle_min=" << pdata->coneseek_angle_min << ", coneseek_angle_max=" << pdata->coneseek_angle_max 
                    << ", coneseek_intlen=" << pdata->coneseek_intlen
                    << ", process_angle=" << pdata->process_angle << ", process_dir=" << pdata->process_dir 
                    << ", process_yaw=" << ioff(pdata->process_yaw, 2) << ", yaw_src=" << yaw_src);
            } else
            /* NAV_ANGLE */
            if (nav_angle < ANGLE_OK) {
                int nava_delay = (timeDelta(minmax_lastt) < 2000) && (nav_angle < CALIB_NAV_ANGLE_MIN) && (nav_angle > CALIB_NAV_ANGLE_MAX);
                /* check obstacle around 1m backwards */
                int nava_noobst = -1;
                if (nava_delay) {
                    double nx = pdata->process_x;
                    double ny = pdata->process_y;
                    double nyaw = pdata->process_yaw;
                    double nt = timeAdd2(pdata->process_time, -2000);
                    update_xy(nx, ny, nyaw, nt, 15, SA_STRAIGHT, VEL_BACK);

                    DegreeMap ndmap;
                    int    namin = -1;
                    int    namax = -1;
                    wmodel.evalGrid(nx, ny, nyaw, ndmap);
                    ndmap.find(DMAP_MIN_INTERVAL_LENGTH, 90, namin, namax);
                    nava_noobst = (namin <= DETECTOBST_ANGLE_MIN) && (namax >= DETECTOBST_ANGLE_MAX);
                    if (nava_noobst) {
                        nava_delay = 0;
                    }
                }
                if (nava_delay  == 12345) {
                    pdata->process_angle = 90;
                    pdata->process_state = PROCESS_STATE_NAV_ANGLE;
                    LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"NAV_ANGLE_DELAY\"): image_number=" << image_number << ", nav_angle=" << ioff(nav_angle, 2) 
                        << ", process_angle=" << pdata->process_angle << ", nava_delay=" << nava_delay << ", nava_noobst=" << nava_noobst
                        << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yawn=" << ioff(yawn, 2) << ", yaw_src=" << yaw_src
                        << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", nav_yaw=" << conf.navigationImuYaw
                        << ", calib.yaw=" << ioff(calib.yaw, 2) << ", calib.azimuth=" << ioff(calib.azimuth, 2));
                } else {
                    // calculate process_angle
                    pdata->process_angle = 90 - trunc(nav_angle);
                    if (pdata->process_angle < NAVIGATION_ANGLE_MIN) {
                        pdata->process_angle = NAVIGATION_ANGLE_MIN;
                    }
                    if (pdata->process_angle > NAVIGATION_ANGLE_MAX) {
                        pdata->process_angle = NAVIGATION_ANGLE_MAX;
                    }
                    pdata->process_state = PROCESS_STATE_NAV_ANGLE;
                    LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"NAV_ANGLE\"): image_number=" << image_number << ", nav_angle=" << ioff(nav_angle, 2) 
                        << ", process_angle=" << pdata->process_angle << ", nava_delay=" << nava_delay << ", nava_noobst=" << nava_noobst
                        << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yawn=" << ioff(yawn, 2) << ", yaw_src=" << yaw_src
                        << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) << ", nav_yaw=" << conf.navigationImuYaw
                        << ", calib.yaw=" << ioff(calib.yaw, 2) << ", calib.azimuth=" << ioff(calib.azimuth, 2));
                }
            }
        }
        /* obstacle detected -> MIN_MAX */
        if ((pdata->process_angle >= (int)ANGLE_OK) && (pdata->process_angle_min >= 0) && (pdata->process_angle_max >= 0)) {
            pdata->process_angle = (pdata->process_angle_min + pdata->process_angle_max)/2; 
            pdata->process_state = PROCESS_STATE_MIN_MAX;
            minmax_lastt = timeBegin();
            LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"MIN_MAX\"): image_number=" << image_number << ", navp_azimuth=" << ioff(pdata->gps_navp_azimuth, 2) 
                << ", process_yaw=" << ioff(pdata->process_yaw, 2) 
                << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max 
                << ", process_angle=" << pdata->process_angle << ", process_dir=" << pdata->process_dir << ", yaw_src=" << yaw_src);
        }
        /* no route found? force wrongway after some time (processing_lag should not cause wrongway) */
        if ((pdata->process_angle >= (int)ANGLE_OK) && (yawn < ANGLE_OK) && (timeDelta(noangle_to) >= PROCESS_NOANGLE_TO)) {
            // (angle > -180) && (angle <= 180)
            double wwf_angle = nav_angle;
            if (nav_angle >= ANGLE_OK) {
                wwf_angle = 0;  // if no navigation goal is defined, go straight backward
            } 
            wrongway_force(wrongway, yawn, wwf_angle);
            if (wrongway_process(wrongway, yawn, pdata->process_angle, pdata->process_velocity, pdata->process_state) > 0) {
                LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"WRONG_WAY_FORCE\"): image_number=" << image_number << ", process_angle=" << pdata->process_angle << ", process_velocity=" << pdata->process_velocity);
            }
        }
        if (pdata->process_angle >= (int)ANGLE_OK) {
            LOGM_INFO(loggerIstro, "process_thread", "process_angle(\"NO_ANGLE\"): image_number=" << image_number << ", process_angle=" << pdata->process_angle << ", process_velocity=" << pdata->process_velocity  
                << ", noangle_dt=" << ((int)timeDelta(noangle_to)));
        } else {
            // log only warning, if it will be usefull could be implemented as noangle -> wrongway
            if ((pdata->process_angle < PROCESS_NOANGLE_MIN) || (pdata->process_angle > PROCESS_NOANGLE_MAX)) {
                LOGM_WARN(loggerIstro, "process_thread", "msg=\"process_noangle_min/max!\", image_number=" << image_number
                    << ", process_angle=" << pdata->process_angle << ", process_velocity=" << pdata->process_velocity  
                    << ", noangle_dt=" << ((int)timeDelta(noangle_to)));
            }
            noangle_to = timeBegin();
        }

        /* determine driving speed/velocity */
        speedctl_process(speedctl, pdata->process_dmap, pdata->process_dir, pdata->process_velocity, pdata->process_state,  pdata->process_stop);
        
        // new GPS data were received? use gps position to update robot position
        if ((pdata->process_ref) && (pdata->gps_ref > 0) && (process_change & PROCESS_CHANGE_GPS) > 0) {
            double dt = timeDelta2(pdata->process_time, pdata->gps_time);  
            LOGM_DEBUG(loggerIstro, "update_xy_gps", "process_x=" << ioff(pdata->process_x, 3) << ", process_y=" << ioff(pdata->process_y, 3)
                 << ", gps_x=" << ioff(pdata->gps_x, 3) << ", gps_y=" << ioff(pdata->gps_y, 3) << ", dt=" << ioff(dt, 2));
            pdata->process_x    = pdata->gps_x;
            pdata->process_y    = pdata->gps_y;            
            pdata->process_time = pdata->gps_time;
        }
        
        // update robot position based on last position (from process_time time)
        update_xy(pdata->process_x, pdata->process_y, pdata->process_yaw, pdata->process_time,
            pdata->ctrlb_ircv500, pdata->ctrlb_angle, pdata->ctrlb_velocity);

        pdata->image_number = image_number++;
            
        if (process_writeData(pdata->image_number, pdata->process_angle, pdata->process_velocity, pdata->process_state, pdata->process_stop,
                pdata->process_dir, pdata->process_ref, pdata->process_x, pdata->process_y, pdata->process_yaw, pdata->coneseek_stop, pdata->coneseek_intlen) < 0) {
            result = -3;
            break;
        }
        
        process_change = 0;
        timeEnd("istro::process_thread.process", t);

        LOGM_INFO(loggerIstro, "process_thread", "msg=\"data processed\", image=" << image_number-1);
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "process_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }
    
    if (result >= 0) {
        LOGM_INFO(loggerIstro, "process_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "process_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}

void saveImage(long image_number, const string& str, const string& ext, const Mat& img)
{
    char filename[256];
//  long tick_number = getTickCount();

    if (img.empty()) return;

    double t = timeBegin();
        
    sprintf(filename, "%srt2019_%07u_%07u_%s.%s", outputDir.c_str(), (unsigned int)save_rand, (unsigned int)(image_number<0?999999:image_number), str.c_str(), ext.c_str());
    imwrite(filename, img);
    sprintf(filename, "rt2019_%07u_%07u_%s.%s", (unsigned int)save_rand, (unsigned int)(image_number<0?999999:image_number), str.c_str(), ext.c_str());
    LOGM_DEBUG(loggerIstro, "saveImage", str.c_str() << "_image=\"" << filename << "\"");
    
    sprintf(filename, "istro::saveImage('%s')", str.c_str());    
    timeEnd(filename, t);
}

int save_readData(long &image_number, double &gps_latitude, double &gps_longitude, double &gps_latitude_raw, double &gps_longitude_raw, 
                  double &gps_navp_dist, double &gps_navp_dist_raw, double &gps_navp_azimuth, double &gps_navp_azimuth_raw, 
                  double &gps_navp_latitude, double &gps_navp_longitude) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    image_number = pdata->image_number;
    gps_latitude = pdata->gps_latitude;
    gps_longitude = pdata->gps_longitude;
    gps_latitude_raw = pdata->gps_latitude_raw;
    gps_longitude_raw = pdata->gps_longitude_raw;
    gps_navp_dist = pdata->gps_navp_dist;
    gps_navp_dist_raw = pdata->gps_navp_dist_raw;
    gps_navp_azimuth = pdata->gps_navp_azimuth;
    gps_navp_azimuth_raw = pdata->gps_navp_azimuth_raw;
    gps_navp_latitude = pdata->gps_navp_latitude;
    gps_navp_longitude = pdata->gps_navp_longitude;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::save_readData", t);
#endif

    return 0;
}

void saveNavMap(const string& str, const string& ext, Mat& img)
{
    int np = 0;
    char filename[256];
    aux_point_t auxpt[3+1];
    
    double t = timeBegin();
    
    long image_number;
    double gps_latitude, gps_longitude, gps_latitude_raw, gps_longitude_raw;
    double gps_navp_dist, gps_navp_dist_raw, gps_navp_azimuth, gps_navp_azimuth_raw;
    double gps_navp_latitude, gps_navp_longitude;
    
    save_readData(image_number, gps_latitude, gps_longitude, gps_latitude_raw, gps_longitude_raw,
                  gps_navp_dist, gps_navp_dist_raw, gps_navp_azimuth, gps_navp_azimuth_raw,
                  gps_navp_latitude, gps_navp_longitude);

    if ((gps_latitude_raw < ANGLE_OK) && (gps_longitude_raw < ANGLE_OK)) {         
        strcpy(auxpt[np].name, "gps_raw");
        sprintf(auxpt[np].desc, "gps_latitude_raw=%0.7f, gps_longitude_raw=%0.7f", gps_latitude_raw, gps_longitude_raw);
        strcpy(auxpt[np].style, "style4");
        auxpt[np].longitude = gps_longitude_raw;
        auxpt[np].latitude  = gps_latitude_raw;
        np++;
    }

    if ((gps_latitude < ANGLE_OK) && (gps_longitude < ANGLE_OK)) {         
        strcpy(auxpt[np].name, "gps");
        sprintf(auxpt[np].desc, "gps_latitude=%0.7f, gps_longitude=%0.7f", gps_latitude, gps_longitude);
        strcpy(auxpt[np].style, "style4");
        auxpt[np].longitude = gps_longitude;
        auxpt[np].latitude  = gps_latitude;
        np++;
    }

    if ((gps_navp_latitude < ANGLE_OK) && (gps_navp_longitude < ANGLE_OK)) {         
        strcpy(auxpt[np].name, "navp");
        sprintf(auxpt[np].desc, "gps_navp_latitude=%0.7f, gps_navp_longitude=%0.7f, gps_navp_dist=%0.3f, gps_navp_azimuth=%0.2f, gps_navp_dist_raw=%0.3f, gps_navp_azimuth_raw=%0.2f", 
                gps_navp_latitude, gps_navp_longitude, gps_navp_dist, gps_navp_azimuth, gps_navp_dist_raw, gps_navp_azimuth_raw);
        strcpy(auxpt[np].style, "style4");
        auxpt[np].longitude = gps_navp_longitude;
        auxpt[np].latitude  = gps_navp_latitude;
        np++;
    }

    auxpt[np].style[0] = 0;

    sprintf(filename, "%srt2019_%07u_%07u_%s.%s", outputDir.c_str(), (unsigned int)save_rand, (unsigned int)(image_number<0?999999:image_number), str.c_str(), ext.c_str());
    navmap_export_kml(filename, auxpt, gps_pt);
    
    navmap_draw(img, auxpt, gps_pt);
    saveImage(image_number, "navmap", "png", img);

    timeEnd("istro::saveNavMap", t);
}

const int SAVE_PERIOD        =   250;  // (in miliseconds) 4 times per second save one camera+vision image, one lidar image and one wmgrid image (it takes max 200ms to save all images)
const int SAVE_PERIOD_WMGRIF = 10000;  // (in miliseconds) every 10 seconds save wmdridf (could take more time)
const int SAVE_PERIOD_NAVMAP =  1000;  // (in miliseconds) every 1 second write kml

void *save_thread(void *parg)
{
    double t;
    int result = 0;
    long last_wmimgnum = -1;
    Mat vision_img;
    Mat lidar_img;
    Mat wmgrid_img;
    Mat wmgrif_img;
    Mat navmap_img;
    
    int save_change = 0;
    int save_camera = 0;  // was camera_img saved during this period
    int save_lidar  = 0;  // was lidar_img saved during this period
    int save_wmodel = 0;  // was wmodel_img saved during this period
    double save_to = -1;
    double wmgrif_to = -1;
    double navmap_to = -1;
    char tname[32];
    
    DataSet *pdata;

    LOG_THREAD_NAME("save");
    LOGM_INFO(loggerIstro, "save_thread", "msg=\"start\"");

    try {

    save_to = timeBegin();
    wmgrif_to = timeBegin();
    navmap_to = timeBegin();
    while (!thread_testcancel()) {
        t = timeBegin();
        pdata = (DataSet *)threads.getData(THDATA_STATE_PROCESSED, THDATA_STATE_SAVING);
        if (pdata == NULL) {
            result = -1;
            break;
        }
        timeEnd("istro::save_thread.wait", t);

        t = timeBegin();
        save_change = 0;        
        if (timeDelta(save_to) >= SAVE_PERIOD) {
            save_camera = 0;
            save_lidar  = 0;
            save_wmodel = 0;
            save_to = timeBegin();
        }

        if (!conf.useNosave) { 
            int write_wmodel = 0;
        
            if ((conf.useCamera) && (!pdata->camera_img.empty()) && !save_camera) {
                save_change += 1;
                save_camera = 1;
                camera.drawFrame(pdata->camera_img);
                saveImage(pdata->image_number, "camera", "jpg", pdata->camera_img);
            
                vision.drawOutput(pdata->camera_img, vision_img, pdata->vision_markers, pdata->vision_epweigth, pdata->vision_elweigth, pdata->vision_angle_min, pdata->vision_angle_max);
                saveImage(pdata->image_number, "vision", "jpg", vision_img);
            }
        
            if ((conf.useLidar) && (pdata->lidar_data_cnt > 0) && !save_lidar) {
                save_change += 2;
                save_lidar = 1;
                lidar.drawOutput(pdata->lidar_data, pdata->lidar_data_cnt, lidar_img, /*pdata->lidar_dmap,*/ pdata->lidar_stop, pdata->lidar_angle_min, pdata->lidar_angle_max, pdata->process_angle, pdata->process_angle_min, pdata->process_angle_max, pdata->image_number);
                saveImage(pdata->image_number, "lidar", "png", lidar_img);
                // write wmodel together with new lidar image!!
                write_wmodel = 1;
            }
            
            if (write_wmodel && (wmodel.image_number != last_wmimgnum) && !save_wmodel){
                save_change += 4;
                save_wmodel = 1;
                last_wmimgnum = wmodel.image_number;
                wmodel.drawGrid(wmgrid_img);                              // fixme: this access to WMGrid is not thread safe!                 
                saveImage(last_wmimgnum, "wmgrid", "png", wmgrid_img);    // storing image with different image_number, because pdata and wmodel are not correlated in time
                if (timeDelta(wmgrif_to) >= SAVE_PERIOD_WMGRIF) {
                    save_change += 10;
                    wmodel.drawGridFull(wmgrif_img);
                    saveImage(last_wmimgnum, "wmgrif", "png", wmgrif_img);
                    wmgrif_to = timeBegin();
                }
                if (timeDelta(navmap_to) >= SAVE_PERIOD_NAVMAP) {
                    save_change += 20;
                    saveNavMap("navmap", "kml", navmap_img);                          // fixme: not thread safe!                
                    navmap_to = timeBegin();
                }
            }
        }
        if (save_change != 0) {
            sprintf(tname, "istro::save_thread.save%d", save_change);
            timeEnd(tname, t);
        }

        threads.setData(pdata, THDATA_STATE_NEW, -1);    //threads.cleanData(THDATA_STATE_SAVED, THDATA_STATE_NEW);
        if (save_change == 0) {
#ifdef SAVETHREAD_LOG_TRACE0
            LOGM_DEBUG(loggerIstro, "save_thread", "msg=\"data ignored\", image_number=" << pdata->image_number << ", save_change=" << save_change);
#endif
        } else
        if ((save_change & 1) == 1) {
            LOGM_DEBUG(loggerIstro, "save_thread", "msg=\"data saved\", image_number=" << pdata->image_number << ", save_change=" << save_change <<
                ", vision_angle_min=" << pdata->vision_angle_min << ", vision_angle_max=" << pdata->vision_angle_max << 
                ", process_angle=" << pdata->process_angle << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max);
        } else
        if ((save_change & 2) == 2) {
            LOGM_DEBUG(loggerIstro, "save_thread", "msg=\"data saved\", image_number=" << pdata->image_number << ", save_change=" << save_change <<
                ", lidar_angle_min=" << pdata->lidar_angle_min << ", lidar_angle_max=" << pdata->lidar_angle_max <<
                ", lidar_stop=" << pdata->lidar_stop << ", lidar_data_cnt=" << pdata->lidar_data_cnt << 
                ", process_angle=" << pdata->process_angle << ", process_angle_min=" << pdata->process_angle_min << ", process_angle_max=" << pdata->process_angle_max);
        } else {
            LOGM_DEBUG(loggerIstro, "save_thread", "msg=\"data saved\", image_number=" << pdata->image_number << ", save_change=" << save_change);
        }
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "save_thread", "msg=\"exception: " << ex.what() << "\"");
        result = -10;
    }

    if (result >= 0) {
        LOGM_INFO(loggerIstro, "save_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro, "save_thread", "msg=\"exit(" << result << ")\"");
    }
    thread_exit();

    return NULL;
}


// automaticky vykonavane steering/velocity programy v manualnom rezime
const int PROG_TYPE_NONE        = 0;      // ziadny program
const int PROG_TYPE_SNAKE       = 1;      // polkruhy dolava a potom doprava
const int PROG_TYPE_BACKFWD     = 2;      // dozadu dopredu

const int PROG_REPEATCMD_PERIOD =   500;  // (v milisekundach) po akom case v kazdom kroku znovu posle prikaz so zmenou rychlosti/smeru (ak nahodou control board predtym prikaz nevykonal)

const int PROG1_STRAIGHT_PERIOD =  6000;  // (v milisekundach) trvanie rovneho useku pred/po samotnym vykonavanim programu (pre kalibraciu gps_course vs euler_x)
const int PROG1_TURN_PERIOD     = 20000;  // (v milisekundach) trvanie 
const int PROG1_TURN_COUNT      =     2;  // pocet vykonanych otoceni     

const int PROG2_FWDBACK_PERIOD  = 7000;  
const int PROG2_WAIT_PERIOD     = 3000;
const int PROG2_WAIT2_PERIOD    =  200;
const int PROG2_RUN_COUNT       =   10;

int prog_type = PROG_TYPE_NONE;
int prog_dir = +1;
int prog_step = 0;
int prog_step_old = -1;
int prog_wait = 0;      // program execution was interrupted by ctrlb_state
int prog_cnt  = 0;
double prog_stept  = 0;
double prog_stept2 = 0;  // resend speed/direction after PROG_REPEATCMD_PERIOD timeout
double prog_waitt  = 0;  // when the program was interrupted

/* start program execution */
void prog_start(int type, int dir)
{
    prog_type = type;
    prog_dir = dir;
    prog_step = 0;
    prog_step_old = -1;
    prog_stept = timeBegin();
    prog_wait = 0;
    prog_cnt = 0;
}

/* stop program execution */
void prog_stop()
{
    prog_type = PROG_TYPE_NONE;
}

/* execute program loop */
int prog_loop(int ctrlb_state, int &angle_change, int &angle, int &velocity_change, int &velocity)
{
    int change = 0;
    int step_old = prog_step;
    //int velocity_old = velocity;
    //int angle_old = angle;
        
    // wait: program execution is interrupted?
    if (CTRLB_STATE_OBSTACLE(ctrlb_state)) {
        if (!prog_wait) {
            LOGM_INFO(loggerIstro, "prog_loop", "msg=\"wait: program was interrupted!\"");
            prog_wait = 1;
            prog_waitt = timeBegin();
            change = 1;
        }
    } else {
        if (prog_wait) {
            LOGM_INFO(loggerIstro, "prog_loop", "msg=\"wait: program will continue...\"");
            prog_wait = 0;
            prog_stept += timeBegin() - prog_waitt;
            change = 1;
        }        
    }

    if (prog_step_old != prog_step) {
        change = 1;
    }
            
    /* SNAKE */
    if ((prog_type == PROG_TYPE_SNAKE) && (!prog_wait)) {
        switch(prog_step) {
        case 0:  // go-straight
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #0 - go-straight\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept) >= PROG1_STRAIGHT_PERIOD) {
                prog_step = 1;
                prog_stept = timeBegin();
            }
            break;
        case 1:  // turn-left
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #1 - turn-left\"");
            }
            angle = SA_MAX;    // S1_MAX je plny lavy
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept) >= PROG1_TURN_PERIOD) {
                if ((++prog_cnt) < PROG1_TURN_COUNT) {
                    prog_step = 2;
                } else {
                    prog_step = 8;
                }
                prog_stept = timeBegin();
            }
            break;
        case 2:  // turn-right
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #2 - turn-right\"");
            }
            angle = SA_MIN;    // S1_MIN je plny pravy
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept) >= PROG1_TURN_PERIOD) {
                if ((++prog_cnt) < PROG1_TURN_COUNT) {
                    prog_step = 1;
                } else {
                    prog_step = 8;
                }
                prog_stept = timeBegin();
            }
            break;
        case 8:  // go-straight
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #8 - go-straight\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept) >= PROG1_STRAIGHT_PERIOD) {
                prog_step = 9;
                prog_stept = timeBegin();
            }
            break;
        case 9:
            LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #9: finished!\"");
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            prog_stop();
            break;
        }
    }
    
    /* BACKFWD */
    if ((prog_type == PROG_TYPE_BACKFWD) && (!prog_wait)) {
        switch(prog_step) {
        case 0:  // go-straight
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #0 - forward\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept) >= PROG2_FWDBACK_PERIOD) {
                prog_step = 1;
                prog_stept = timeBegin();
            }
            break;
        case 1:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #1 - wait\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept) >= PROG2_WAIT_PERIOD) {
                prog_step = 2;
                prog_stept = timeBegin();
            }
            break;
        case 2:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #2 - wait2\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityBack;
            if (timeDelta(prog_stept) >= PROG2_WAIT2_PERIOD) {
                prog_step = 3;
                prog_stept = timeBegin();
            }
            break;

        case 3:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #3 - wait3\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept) >= PROG2_WAIT2_PERIOD) {
                prog_step = 4;
                prog_stept = timeBegin();
            }
            break;
        case 4:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #4 - wait4\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityBack;
            if (timeDelta(prog_stept) >= PROG2_WAIT2_PERIOD) {
                prog_step = 5;
                prog_stept = timeBegin();
            }
            break;
        case 5:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #5 - wait5\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept) >= PROG2_WAIT2_PERIOD) {
                prog_step = 6;
                prog_stept = timeBegin();
            }
            break;

        case 6:  // go-backward
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #6 - back\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityBack;
            if (timeDelta(prog_stept) >= PROG2_FWDBACK_PERIOD) {
                prog_step = 7;
                prog_stept = timeBegin();
            }
            break;
        case 7:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #6 - wait\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept) >= PROG2_WAIT_PERIOD) {
                if ((++prog_cnt) < PROG2_RUN_COUNT) {
                    prog_step = 0;
                } else {
                    prog_step = 9;
                }
                prog_stept = timeBegin();
            }
            break;
        case 9:
            LOGM_INFO(loggerIstro, "prog_loop", "msg=\"step #9: finished!\"");
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            prog_stop();
            break;
        }
    }

    if (change) {
        prog_stept2 = prog_stept;
    }
    if ((timeDelta(prog_stept2) >= PROG_REPEATCMD_PERIOD) && (!prog_wait)) {
        change = 2;
        prog_stept2 = timeBegin(); 
        LOGM_DEBUG(loggerIstro, "prog_loop", "change=" << change);
    }

    velocity_change = change;  //(velocity != velocity_old) || change;
    angle_change = change;     //(angle != angle_old) || change;

    if (change == 1) {
        LOGM_INFO(loggerIstro, "prog_loop", "prog_type=" << prog_type << ", prog_step_old=" << prog_step_old << ", prog_step=" << prog_step << 
            ", prog_cnt=" << prog_cnt << ", prog_wait=" << prog_wait << ", angle=" << angle << ", velocity=" << velocity << ", change=" << change);
    }

    prog_step_old = step_old;
    
    return change;
}

int ppoint = 0;
int ledprg = 0;

int loop_waitKey(int delayms, int &steering_change, int &steering_type, int &steering_angle, 
        int &velocity_change, int &velocity, int &velocity_stop, int process_velocity, double gps_latitude, double gps_longitude)
{
    char c = (char) waitKey(delayms);
    if ((c == 27) || (c == 'x') || (c == 'X')) {
        return 1;
    }
    
    switch(c) {
    case 'l':
    case 'L':
        steering_type = MANUAL;
        if (++ledprg > 4) ledprg = 0;
        ctrlBoard.setLedProgram(ledprg);
        break;
    case 'g':
    case 'G':
        //print GPS position
        printf("ppoint[%d]: gps_latitude=%.7f, gps_longitude=%.7f\n", ppoint, gps_latitude, gps_longitude);
        LOGM_INFO(loggerIstro, "loop_waitKey", "ppoint=" << ppoint 
            << ", gps_latitude=" << ioff(gps_latitude, 7) << ", gps_longitude=" << ioff(gps_longitude, 7)); 
        ppoint++;
        break;
    case 'q':
    case 'Q':
        //left
        steering_type = MANUAL;
        steering_angle = SA_MAX;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'e':
    case 'E':
        //right
        steering_type = MANUAL;
        steering_angle = SA_MIN;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'a':
    case 'A':
        //left
        steering_type = MANUAL;
        steering_angle += 10;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'd':
    case 'D':
        //right
        steering_type = MANUAL;
        steering_angle -= 10;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'w':
    case 'W':
        velocity++;
        if ((velocity_stop > 0) || (process_velocity >= 0)) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstro, "loop_waitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstro, "loop_waitKey", "msg=\"velocity++\", velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    case 's':
    case 'S':
        velocity--;
        if ((velocity_stop > 0) || (process_velocity >= 0)) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstro, "loop_waitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstro, "loop_waitKey", "msg=\"velocity--\", velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    case ' ':
        //Stop
        steering_type = MANUAL;
        steering_angle = SA_STRAIGHT;
        steering_change = 1;
        velocity = VEL_ZERO;
        velocity_change = 1;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "msg=\"stop!\"");
        break;
    case 'm':
    case 'M':
        //Manual steering
        steering_type = MANUAL;
        prog_stop();
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_type=\"MANUAL\"");
        break;
    case 'u':
    case 'U':
        //Autonomous steering
        steering_type = AUTONOMOUS;
        LOGM_INFO(loggerIstro, "loop_waitKey", "steering_type=\"AUTONOMOUS\"");
        break;
    case 'p':
    case 'P':
        //Program
        steering_type = MANUAL;
        LOGM_INFO(loggerIstro, "loop_waitKey", "msg=\"start program!\", steering_type=\"MANUAL\"");
        prog_start(PROG_TYPE_BACKFWD, +1);
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
        velocity = VEL_ZERO + (c - '0') * 3;
        if ((velocity_stop > 0) || (process_velocity >= 0)) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstro, "loop_waitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstro, "loop_waitKey", "velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    default:
        ;
    }

    return 0;    
}

int loop_readData(int &process_angle, int &process_velocity, int &process_state, int &process_stop, int &ctrlb_state, 
        int &gps_fix, double &gps_latitude, double &gps_longitude, double &gps_course) 
{
    DataSet *pdata;
    
#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    process_angle = pdata->process_angle;
    process_velocity = pdata->process_velocity;
    process_state = pdata->process_state;
    process_stop = pdata->process_stop;
    ctrlb_state = pdata->ctrlb_state;
    gps_fix = pdata->gps_fix;
    gps_latitude = pdata->gps_latitude;
    gps_longitude = pdata->gps_longitude;
    gps_course = pdata->gps_course;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstro, "loop_readData", "process_angle=" << process_angle 
        << ", process_velocity=" << process_velocity << ", process_state=" << process_state << ", process_stop=" << process_stop 
        << ", ctrlb_state=" << ctrlb_state << ", gps_fix=" << gps_fix << ", gps_course=" << ioff(gps_course, 2));
        
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::loop_readData", t);
#endif

    return 0;
}

const int VELOCITY_SET_PERIOD = 3000;  // (v milisekundach) ako casto posielat rychlost do CB, aj ked sa nemeni; 0 = nikdy 
const int DISPLAY_PERIOD      =  500;  // (v milisekundach) ako casto vypisovat na LCD displej stav; 0 = nikdy 
//const int VELOCITY_HIGH_PERIOD = 5000;  // (v milisekundach) po kolkych sekundach zvysit rychlost, ak sme stale v stave PROCESS_STATE_NAV_ANGLE?

int loop(void)
{
//    int process_state_last = -1;

    int steering_change = 0;
    int steering_type = AUTONOMOUS;
    int steering_angle = SA_STRAIGHT;
    int steering_type_old = steering_type;
    
    int velocity_change = 1;
    int velocity = VEL_ZERO + conf.velocityFwd;         // initial speed
    //int velocity_high = VEL_ZERO + conf.velocityFwdHigh;    // high speed
    int velocity_stop = 0;
    double velocity_lastt = timeBegin();
    //double velocity_hight = timeBegin();  /* ked presiahne hodnotu VELOCITY_HIGH_PERIOD */
    
    double display_lastt = timeBegin();
    double led_to = timeBegin();
    double balldrop_lastt = -1;

    if (conf.useControlBoard) {
        sleep(3);
        if (conf.ControlBoardInitStr) {
            ctrlBoard.setLedProgram(1);
            ctrlBoard.writeString(conf.ControlBoardInitStr);
        }
//      ctrlBoard.start();
    }

    LOGM_INFO(loggerIstro, "loop", "msg=\"start\"");

    try {
    
#ifdef ISTRO_GUI
    if ((!conf.useCamera) && (!conf.useLidar)) 
#endif        
    {
        // fixme: empty window for waitKey() - waitKey() does not work without any other windows
        Mat img_empty(100, 100, CV_8UC3, Scalar( 0,0,0) );
        imshow("empty", img_empty);
    }
    
    if (threads.start(capture_camera_thread, capture_lidar_thread, gps_thread, ahrs_thread, ctrlBoard_thread, vision_thread, process_thread, save_thread) < 0) {
        return -1;
    }

    /* pocas Robotouru 2017 sme zistili problem pri nastavovani thresholdov pre ultrazvuky - asi problem RSky v Arduine */
    /* pred spustenim threadov nevycitavame RSku, takze nevieme korektne parsovat odpovede - skusime rovnaky test tu */
    /* pozor: zdrzanie 3 sekundy!  -> pri dalsich testoch 8.5.2018 prisli vzdy spravne odpovede z Arduina => asi bude OK */
    //if (conf.useControlBoard) {
    //    //ctrlBoard.writeString("U4001");
    //    //ctrlBoard.writeString("U5001");
    //    sleep(1);
    //    for(int i=0; i<10; i++) {
    //        ctrlBoard.setXX4();
    //        msleep(100);
    //    }
    //    for(int i=0; i<10; i++) {
    //        ctrlBoard.setXX5();
    //        msleep(100);
    //    }
    //}

    const char *pstate_str = NULL;
    for(;;) {
        steering_change = 0;
        
        int process_stop = -1;
        int process_state = -1;
        int process_angle = (int)ANGLE_NONE;
        int process_velocity = -1;
        int ctrlb_state = 0;
        int gps_fix = -1;
        double gps_latitude = 0;
        double gps_longitude = 0; 
        double gps_course = 0;

        if (loop_readData(process_angle, process_velocity, process_state, process_stop, ctrlb_state, gps_fix, gps_latitude, gps_longitude, gps_course) < 0) {
            break;
        }
        
        if (steering_type == AUTONOMOUS) {
            if (process_angle < (int)ANGLE_OK) {
                steering_angle = (int)((process_angle - 90) * SA_MULT) + SA_STRAIGHT;
                if (conf.useControlBoard) {
                    ctrlBoard.setSteeringAngle(steering_angle);
                }
            }
            
            int velocity_set = 0;    // poslat rychlost, aj ked sa nemeni jej velkost?
            if ((VELOCITY_SET_PERIOD > 0) && (timeDelta(velocity_lastt) >= VELOCITY_SET_PERIOD)) {
                velocity_set = 1;
            }

            /* set velocity calculated by process_thread */
            if ((process_stop <= 0) && (process_velocity >= 0)) {
                // send new velocity to control board only if there is no obstacle 
                if (!ctrlb_obstState(ctrlb_state, process_velocity) && (ctrlb_state != CTRLB_STATE_EBTN)) {
                    velocity_stop = 0;
                    velocity_lastt = timeBegin();
                    if (conf.useControlBoard) {
                        ctrlBoard.setSpeed(process_velocity);
                        // velocity = process_velocity;
                    }
                }
            } else
            /* perform stop (if process_stop is forced) or resend last velocity */
            if (process_stop >= 0) {
                if (!ctrlb_obstState(ctrlb_state, velocity) && (ctrlb_state != CTRLB_STATE_EBTN)) {
                    if ((process_stop > 0) && ((velocity_stop == 0) || velocity_set)) {
                        velocity_stop = 1;
                        velocity_lastt = timeBegin();
                        if (conf.useControlBoard) {
                            //ctrlBoard.stop();
                            ctrlBoard.setSpeed(VEL_ZERO);
                            ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_RED);    // red: wrongway / process_stop
                        }
                    }
                    if ((process_stop == 0) && ((velocity_stop > 0) || velocity_set)) {
                        velocity_stop = 0;
                        velocity_lastt = timeBegin();
                        if (conf.useControlBoard) {
                            //if (timeDelta(velocity_hight) >= VELOCITY_HIGH_PERIOD) {
                            //   ctrlBoard.setSpeed(velocity_high);
                            //} else 
                            ctrlBoard.setSpeed(velocity);
                        }
                    }
                }
            }

            /* BALL_DROP - kazdych par sekund posli pocet lopticiek, ktore uz maju byt vylozene */
            if ((process_state >= PROCESS_STATE_BALLDROP) && (process_state <= PROCESS_STATE_BALLDROP_9)) {
                if ((balldrop_lastt < 0) || (timeDelta(balldrop_lastt) > 350)) {
                    balldrop_lastt = timeBegin();
                    ctrlBoard.setBallDrop(process_state - PROCESS_STATE_BALLDROP);
                }
            } else {
                balldrop_lastt = -1;
            }
            
            // fixme: nefungovalo nikdy nenastala podmienka pre velocity_high (doplnit logovanie, zmiernit podmienky)
            //if ((process_state != PROCESS_STATE_NAV_ANGLE) || (process_stop >= 0) || (process_velocity >= 0)) {
            //    velocity_hight = timeBegin();
            //}
            
            /* update LED state */
            if ((timeDelta(led_to) > 200) && (process_stop <= 0)) {
                led_to = timeBegin();
//                process_state_last = process_state;
                if (conf.useControlBoard) {
                    if (process_state == PROCESS_STATE_NONE) {
                        pstate_str = NULL;
                        ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_WHITE);
                    } else 
                    if (process_state == PROCESS_STATE_CALIBRATION) {
                        pstate_str = "CALB";
                        ctrlBoard.setLedMask(255, 255, 255, 0, 0);            // yellow
                    } else 
                    if (process_state == PROCESS_STATE_WRONGWAY) {
                        pstate_str = "WROW";
                        ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_RED);       // red: wrongway / process_stop
                    } else 
                    if (process_state == PROCESS_STATE_LOADING) {
                        pstate_str = "LOAD";
                        ctrlBoard.setLedMask(255, 255, 0, 255, 0);            // magenta
                    } else 
                    if (process_state == PROCESS_STATE_UNLOADING) {
                        pstate_str = "UNLD";
                        ctrlBoard.setLedMask(255, 0, 255, 255, 0);            // cyan
                    } else 
                    if (process_state == PROCESS_STATE_CONE_SEEK) {
                        pstate_str = "CONE";
                        ctrlBoard.setLedMask(255, 255, 0, 255, 0);            // magenta
                    } else
                    if ((process_state >= PROCESS_STATE_BALLDROP) && (process_state <= PROCESS_STATE_BALLDROP_9)) {
                        pstate_str = "BALL";
                        // fixme: wait before sending the command, to avoid arduino to miss it
                        msleep(30);
                        ctrlBoard.setLedMask(255, 0, 255, 255, 0);            // cyan
                    } else 
                    if (process_state == PROCESS_STATE_NAV_ANGLE) {
                        pstate_str = "NAVA";
                        ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_GREEN);
                    } else 
                    if (process_state == PROCESS_STATE_MIN_MAX) {
                        pstate_str = "MMAX";
                        ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_BLUE);
                    } else 
                    if (process_state == PROCESS_STATE_QRSCAN_COORD) {
                        pstate_str = "QRSC";
                        ctrlBoard.setLedMask(255, 255, 0, 255, 1);            // magenta+blink
                    }
                }
            }
        }
        
        /* osetrime stlacenie klavesov */
        if (loop_waitKey(20, steering_change, steering_type, steering_angle, 
                velocity_change, velocity, velocity_stop, process_velocity, gps_latitude, gps_longitude) > 0) {
            break;
        }

        if ((steering_type == MANUAL) && (steering_type_old != steering_type)) {
            ctrlBoard.setLedProgram(CTRLB_LED_PROGRAM_OFF);
        }
        steering_type_old = steering_type;

        /* vykonavanie automatickeho steering/vehicle programu */
        if ((steering_type == MANUAL) && (prog_type != PROG_TYPE_NONE)) {
            prog_loop(ctrlb_state, steering_change, steering_angle, velocity_change, velocity);
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

        if ((DISPLAY_PERIOD > 0) && (timeDelta(display_lastt) >= DISPLAY_PERIOD)) {
            char ss[30];
            display_lastt = timeBegin();
            /* vypis stavu vo formate "Af: S0 V098 A092 G180" */
            int vv = ((steering_type == AUTONOMOUS)&&(process_velocity>=0))?process_velocity:velocity;
            sprintf(ss, "%c%c:S%dv%03d A%03d %s", 
                (steering_type == AUTONOMOUS)?('A'):((prog_type != PROG_TYPE_NONE)?('P'):('M')), 
                (ctrlb_state == CTRLB_STATE_EBTN)?('e'):((ctrlb_obstState(ctrlb_state, vv))?'o':'f'), 
                velocity_stop, 
                vv, 
                steering_angle, 
                ((process_stop > 0)?("STOP"):((pstate_str == NULL)?("-"):(pstate_str))));
            if (conf.useControlBoard) {
                ctrlBoard.displayText(ss);
            }
        }
    }

    } catch (std::exception& ex) {
        LOGM_ERROR(loggerIstro, "loop", "msg=\"exception: " << ex.what() << "\"");
    }
    
    threads.stop();

    if (conf.useControlBoard) {
        ctrlBoard.setSpeed(VEL_ZERO);
        ctrlBoard.setSteeringAngle(SA_STRAIGHT);
        ctrlBoard.displayText("bye...");
        sleep(1);
    }

    destroyAllWindows();
    
    LOGM_INFO(loggerIstro, "loop", "msg=\"exit(0)\"");

    return 0;
}

int main(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstro, "-----------------------------");
    LOGM_INFO(loggerIstro, "main", "msg=\"application start\"");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstro, "main", "msg=\"conf.parseArguments() failed!\"");
        return -1;
    }
    conf.printArguments();

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstro, "main", "msg=\"initDevices() failed!\"");
        return -1;
    }
    
    waitForStart();

    loop();
 
    closeDevices();
    
    LOGM_INFO(loggerIstro, "main", "msg=\"application exit\"");
    LOG_DESTROY();
    return 0;
}
