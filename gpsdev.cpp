#include <math.h>
#include <iostream>
#include "gpsdev.h"
#include "system.h"
#include "logger.h"

using namespace std;

LOG_DEFINE(loggerGpsDev, "GpsDevice");

#ifdef ISTRO_GPSD_CLIENT

#include <libgpsmm.h>

int GpsDevice::init(void) 
{   
    gpsmm *pgps;

    pgps = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    pgps_ = (void *)pgps;

    if (pgps->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        LOGM_ERROR(loggerGpsDev, "init", "GPSD is not running!");
        return -1;
    }

    return 0;
}

void GpsDevice::close(void)
{
    gpsmm *pgps = (gpsmm *)pgps_;

    delete pgps;
    pgps_ = NULL;
} 

int GpsDevice::getData(double &latitude, double &longitude, double &speed, double &course)
{
    gpsmm *pgps = (gpsmm *)pgps_;

    struct gps_data_t* pdata;

    if (!pgps->waiting(50000000)) {
        LOGM_ERROR(loggerGpsDev, "getData", "waiting failed!");
        return -1;
    }

    if ((pdata = pgps->read()) == NULL) {
        LOGM_ERROR(loggerGpsDev, "getData", "read failed!");
        return -2;
    }

    int no_fix = (pdata->status == STATUS_NO_FIX) || (pdata->fix.mode == MODE_NOT_SEEN) || (pdata->fix.mode == MODE_NO_FIX);

    /* track = course made good (relative to true north) */
    LOGM_TRACE(loggerGpsDev, "getData", "fix=" << (int)(!no_fix) << ", lat=" << ioff(pdata->fix.latitude, 6) << ", lon=" << ioff(pdata->fix.longitude, 6) 
        << ", speed=" << ioff(pdata->fix.speed, 3) << ", course=" << ioff(pdata->fix.track, 2) 
        << ", sat.used=" << (int)pdata->satellites_used << ", sat.visible=" << (int)pdata->satellites_visible);    
    
    // isnan(pdata->fix.latitude) || isnan(pdata->fix.longitude)
    if (no_fix) {
        latitude = 0;
        longitude = 0;
        speed = 0;
        course = 0;
        return 0;
    }

    // valid fix
    latitude = pdata->fix.latitude;
    longitude = pdata->fix.longitude;
    speed = pdata->fix.speed;
    course = pdata->fix.track;
    return 1;
}

#else 

#include "geocalc.h"

int GpsDevice::init(void) 
{   
    LOGM_WARN(loggerGpsDev, "init", "MOCK GPSDEVICE implementation, DEBUG ONLY!!");
    return 0;
}

void GpsDevice::close(void)
{
} 

int getdata_cnt = 0;

int GpsDevice::getData(double &latitude, double &longitude, double &speed, double &course)
{
    msleep(1000);
    getdata_cnt++;

    // { "N2", 18.743636933, 49.213718281 }  ->  { "M4", 18.744029433, 49.212597581 },
    double p1lo = 18.743636933;
    double p1la = 49.213718281;
    double p2lo = 18.744029433;
    double p2la = 49.212597581;
    int dd = 3 * 130;    // distance between p1 and p2 is 130 metres -> 3x to achieve speed 0.33 m/s
    
    if (getdata_cnt < dd) {
        latitude =  p1la + (p2la - p1la) * getdata_cnt / dd;
        longitude = p1lo + (p2lo - p1lo) * getdata_cnt / dd;
        speed = 0.33;
        course = 167;    // real course N2 -> M4; course is taken from controlboard BNOEVC
    } else {
        latitude =  p2la;
        longitude = p2lo;
    }

/*    
    if (getdata_cnt < 5) {
//  { "E2", 17.1139765, 48.1340145 },
        latitude =  48.1340145;
        longitude = 17.1139765;
        speed = 0.3;
        course = 145;
    } else
    if (getdata_cnt < 15) {
//  { "E2", 17.1139765, 48.1340145 },
        latitude =  48.1340145;
        longitude = 17.1139765;
        speed = 0.3;
        course = 135;  // course changed!
    } else 
    if (getdata_cnt < 25) {
//  { "E6", 17.1124889, 48.1352738 },
        latitude =  48.1352738 - 30 * GEOCALC_LATITUDE_1M;
        longitude = 17.1124889 + 30 * GEOCALC_LONGITUDE_1M;
        speed = 0.3;
        course = 135;
    } else 
    if (getdata_cnt < 35) {
//  { "E6", 17.1124889, 48.1352738 },
        latitude =  48.1352738 - 11 * GEOCALC_LATITUDE_1M;
        longitude = 17.1124889 + 11 * GEOCALC_LONGITUDE_1M;
        speed = 0.3;
        course = 135;
    } else 
    if (getdata_cnt < 45) {
//  { "E6", 17.1124889, 48.1352738 },
        latitude =  48.1352738 - 4 * GEOCALC_LATITUDE_1M;
        longitude = 17.1124889 + 4 * GEOCALC_LONGITUDE_1M;
        speed = 0.3;
        course = 135;    
    } else 
    if (getdata_cnt < 55) {
//  { "M2", 17.1098255, 48.1352596 },
        latitude =  48.1352596;
        longitude = 17.1098255;
        speed = 0.3;
        course = 135;
    } else 
    if (getdata_cnt < 65) {
//  { "M2", 17.1098255, 48.1352596 },
        latitude =  48.1352596;
        longitude = 17.1098255 - 10 * GEOCALC_LONGITUDE_1M;
        speed = 0.3;
        course = 135;
    } else {
//  { "M2", 17.1098255, 48.1352596 },
        latitude =  48.1352596;
        longitude = 17.1098255 - 30 * GEOCALC_LONGITUDE_1M;
        speed = 0.4;
        course = 135;
    }
*/    
    return 1;
}

#endif
