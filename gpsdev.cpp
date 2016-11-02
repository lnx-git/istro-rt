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

int GpsDevice::getData(double &latitude, double &longitude, double &course)
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
        course = 0;
        return 0;
    }

    // valid fix
    latitude = pdata->fix.latitude;
    longitude = pdata->fix.longitude;
    course = pdata->fix.track;
    return 1;
}

#else 

int GpsDevice::init(void) 
{   
    LOGM_WARN(loggerGpsDev, "init", "MOCK GPSDEVICE implementation, DEBUG ONLY!!");
    return 0;
}

void GpsDevice::close(void)
{
} 

int getdata_cnt = 0;

int GpsDevice::getData(double &latitude, double &longitude, double &course)
{
    msleep(1000);
    if (getdata_cnt++ < 10) {
        latitude =  48.830415;
        longitude = 12.955187;
        course = 0;
    } else {
        latitude =  48.831266;
        longitude = 12.953995;
        course = 0;
    }
    return 1;
}

#endif
