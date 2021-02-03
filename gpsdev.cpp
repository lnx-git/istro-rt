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

typedef struct {
    double lat; 
    double lon;
    double speed;
    double course;
} gps_data_t;

#ifdef ISTRO_MAP_BA_FEISTU2
const int    GPSDEV_DATA_COUNT = 44;

// 190425_Istrobot_fei_test\out_1703_tam_a_spat_velka_otocka_zastavil_nakladka 
const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    // navmap_image="ib2019_5370154_0005527_navmap.png"
    { 48.154088, 17.072490, 1.250, 350.42 },
    // navmap_image="ib2019_5370154_0005576_navmap.png"
    { 48.154098, 17.072490, 1.214, 356.41 },
    // navmap_image="ib2019_5370154_0005624_navmap.png"
    { 48.154108, 17.072490, 1.250, 359.96 },
    // navmap_image="ib2019_5370154_0005673_navmap.png"
    { 48.154118, 17.072490, 1.235, 0.84 },
    { 48.154128, 17.072490, 1.235, 357.47 },
    // navmap_image="ib2019_5370154_0005765_navmap.png"
    { 48.154140, 17.072490, 1.235, 358.85 },
    { 48.154150, 17.072488, 1.194, 359.87 },
    // navmap_image="ib2019_5370154_0005817_navmap.png"
    { 48.154162, 17.072488, 1.199, 359.65 },
    // navmap_image="ib2019_5370154_0005872_navmap.png"
    { 48.154173, 17.072490, 1.224, 2.66 },
    // navmap_image="ib2019_5370154_0005919_navmap.png"
    { 48.154183, 17.072490, 1.194, 2.97 },
    // navmap_image="ib2019_5370154_0005970_navmap.png"
    { 48.154195, 17.072492, 1.178, 4.08 },
    // navmap_image="ib2019_5370154_0006041_navmap.png"
    { 48.154205, 17.072490, 1.240, 354.05 },
    // navmap_image="ib2019_5370154_0006089_navmap.png"
    { 48.154217, 17.072490, 1.235, 359.27 },
    { 48.154227, 17.072490, 1.194, 4.11 },
    // navmap_image="ib2019_5370154_0006137_navmap.png"
    { 48.154238, 17.072492, 1.219, 359.60 },
    // navmap_image="ib2019_5370154_0006185_navmap.png"
    { 48.154248, 17.072492, 1.194, 354.03 },
    { 48.154260, 17.072490, 1.209, 0.34 },
    // navmap_image="ib2019_5370154_0006286_navmap.png"
    { 48.154270, 17.072488, 1.157, 336.69 },
    // navmap_image="ib2019_5370154_0006336_navmap.png"
    { 48.154277, 17.072480, 1.121, 303.78 },
    // navmap_image="ib2019_5370154_0006391_navmap.png"
    { 48.154282, 17.072467, 1.106, 288.29 },
    // navmap_image="ib2019_5370154_0006445_navmap.png"
    { 48.154283, 17.072453, 1.008, 284.33 },
    { 48.154285, 17.072442, 0.947, 284.22 },
    // navmap_image="ib2019_5370154_0006510_navmap.png"
    { 48.154287, 17.072430, 0.864, 285.50 },
    // navmap_image="ib2019_5370154_0006566_navmap.png"
    { 48.154287, 17.072423, 0.782, 287.34 },
    // navmap_image="ib2019_5370154_0006619_navmap.png"
    { 48.154290, 17.072413, 0.710, 285.43 },
    // navmap_image="ib2019_5370154_0006673_navmap.png"
    { 48.154292, 17.072403, 0.689, 285.77 },
    // navmap_image="ib2019_5370154_0006730_navmap.png"
    { 48.154295, 17.072393, 0.694, 288.00 },
    { 48.154297, 17.072387, 0.643, 273.44 },
    // navmap_image="ib2019_5370154_0006805_navmap.png"
    { 48.154297, 17.072380, 0.648, 243.74 },
    // navmap_image="ib2019_5370154_0006855_navmap.png"
    { 48.154292, 17.072373, 0.705, 218.95 },
    // navmap_image="ib2019_5370154_0006903_navmap.png"
    { 48.154287, 17.072368, 0.792, 200.62 },
    // navmap_image="ib2019_5370154_0006952_navmap.png"
    { 48.154278, 17.072367, 0.890, 183.34 },
    // navmap_image="ib2019_5370154_0007000_navmap.png"
    { 48.154267, 17.072367, 0.972, 168.64 },
    // navmap_image="ib2019_5370154_0007048_navmap.png"
    { 48.154257, 17.072365, 0.998, 172.50 },
    // navmap_image="ib2019_5370154_0007097_navmap.png"
    { 48.154247, 17.072367, 1.075, 154.00 },
    // navmap_image="ib2019_5370154_0007149_navmap.png"
    { 48.154237, 17.072375, 1.163, 133.18 },
    { 48.154227, 17.072385, 1.163, 136.43 },
    // navmap_image="ib2019_5370154_0007211_navmap.png"
    { 48.154217, 17.072397, 1.127, 144.79 },
    // navmap_image="ib2019_5370154_0007259_navmap.png"
    { 48.154207, 17.072407, 1.080, 148.12 },
    // navmap_image="ib2019_5370154_0007328_navmap.png"
    { 48.154197, 17.072418, 0.921, 146.09 },
    { 48.154188, 17.072428, 0.900, 154.41 },
    // navmap_image="ib2019_5370154_0007390_navmap.png"
    { 48.154182, 17.072437, 0.905, 137.78 },
    // navmap_image="ib2019_5370154_0007445_navmap.png"
    { 48.154175, 17.072450, 0.957, 146.38 },
    // navmap_image="ib2019_5370154_0007498_navmap.png"
    { 48.154167, 17.072460, 0.967, 155.72 }
};
#endif

#ifdef ISTRO_MAP_BA_SADJK
const int    GPSDEV_DATA_COUNT = 1;

// Sad JK
const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 48.1337423, 17.1112223, 0.442, 294.28 }
};
#endif

#ifdef ISTRO_MAP_BA_NABREZIE
const int    GPSDEV_DATA_COUNT = 1;

const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 48.1399831, 17.1165786, 0.442, 294.28 }
};
#endif

#ifdef ISTRO_MAP_MLAZNE_HAMRNIKY
const int    GPSDEV_DATA_COUNT = 1;

// park Hamrniky - Marianske Lazne, Czech Republic
const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    //{ 49.948951, 12.698000, 0.442, 294.28 },   // Start
    //{ (49.948951 + 49.948960)/2.0, (12.698000+12.698260)/2.0, 0.442, 294.28 }    // below 10m to "V3", test: -path "V3M1"
    //{ 49.948960, 12.698260, 0.442, 294.28 }    // "V3", bod 23
    { 49.9520069, 12.6955229, 0.442, 294.28 }    // "Z1", 5339231997
};
#endif

#if defined(ISTRO_MAP_PISEK_PALSADY) || defined(ISTRO_MAP_PISEK_PALSADY2)
const int    GPSDEV_DATA_COUNT = 40;

// 190510_RobotemRovne_test\out_1945_casto_biele_tam_obchadzal
const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 49.308897, 14.151278, 0.442, 294.28 },
    { 49.308898, 14.151273, 0.432, 295.72 },
    { 49.308900, 14.151268, 0.334, 295.51 },
    { 49.308902, 14.151265, 0.473, 291.84 },
    { 49.308902, 14.151257, 0.550, 290.36 },
    { 49.308905, 14.151252, 0.432, 294.42 },
    { 49.308907, 14.151247, 0.406, 304.58 },
    { 49.308908, 14.151238, 0.890, 286.67 },
    { 49.308910, 14.151227, 0.977, 287.53 },
    { 49.308913, 14.151215, 0.715, 289.75 },
    { 49.308915, 14.151205, 0.751, 288.50 },
    { 49.308918, 14.151195, 0.808, 283.68 },
    { 49.308920, 14.151183, 0.895, 279.32 },
    { 49.308922, 14.151172, 0.859, 290.09 },
    { 49.308925, 14.151162, 0.885, 288.81 },
    { 49.308928, 14.151150, 0.859, 301.00 },
    { 49.308933, 14.151140, 0.803, 302.82 },
    { 49.308937, 14.151130, 0.905, 286.71 },
    { 49.308937, 14.151115, 1.137, 277.83 },
    { 49.308940, 14.151102, 1.019, 293.32 },
    { 49.308945, 14.151088, 1.055, 295.84 },
    { 49.308948, 14.151073, 1.008, 295.58 },
    { 49.308953, 14.151060, 1.049, 288.13 },
    { 49.308958, 14.151047, 1.029, 297.30 },
    { 49.308963, 14.151035, 0.977, 285.38 },
    { 49.308967, 14.151022, 1.091, 296.25 },
    { 49.308970, 14.151008, 1.029, 287.17 },
    { 49.308972, 14.150995, 0.844, 281.71 },
    { 49.308973, 14.150983, 0.890, 290.56 },
    { 49.308975, 14.150972, 0.921, 296.44 },
    { 49.308978, 14.150962, 0.694, 296.55 },
    { 49.308982, 14.150952, 0.808, 296.22 },
    { 49.308985, 14.150942, 0.828, 296.62 },
    { 49.308987, 14.150930, 0.844, 295.73 },
    { 49.308990, 14.150918, 0.900, 297.24 },
    { 49.308992, 14.150907, 0.844, 294.47 },
    { 49.308995, 14.150895, 0.890, 296.50 },
    { 49.308997, 14.150883, 0.844, 292.33 },
    { 49.308998, 14.150872, 0.808, 292.32 },
    { 49.308998, 14.150858, 0.787, 284.48 }
};
#endif

#ifdef ISTRO_MAP_DEGGENDORF
const int    GPSDEV_DATA_COUNT = 1;

// Stadthallenpark, Donaupark - Deggendorf, Germany
const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 48.831143, 12.957402, 0.442, 294.28 }  //  "X8"
};
#endif

#ifdef ISTRO_MAP_KLONDAJK
const int    GPSDEV_DATA_COUNT = 1;

const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 49.845089, 14.1606168, 0.442, 294.28 }
};
#endif

#ifdef ISTRO_MAP_BA_PARKAH
const int    GPSDEV_DATA_COUNT = 1;

const gps_data_t gps_data[GPSDEV_DATA_COUNT] = 
{
    { 48.1565166, 17.1554347, 0.442, 294.28 }    // W1
};
#endif


int getdata_cnt = 0;

int GpsDevice::getData(double &latitude, double &longitude, double &speed, double &course)
{
    latitude = gps_data[getdata_cnt].lat;
    longitude = gps_data[getdata_cnt].lon;
    speed = gps_data[getdata_cnt].speed;
    course = gps_data[getdata_cnt].course;

    msleep(1000);
    if (getdata_cnt < GPSDEV_DATA_COUNT - 1) {
        getdata_cnt++;
    }

/*
    // Ostredky:  { "N1", 17.165371, 48.161540 }  ->   { "N4", 17.168001, 48.161734 }
    double p1lo = 17.165371;
    double p1la = 48.161540;
    double p2lo = 17.168001;
    double p2la = 48.161734;
    int dd = 3 * 197;    // distance between p1 and p2 is 197 metres -> 3x to achieve speed 0.33 m/s
    // course = 167;    // real course N2 -> M4; course is taken from controlboard BNOEVC
*/

    // lat=49.213715, lon=18.743650, gps_x=0.24, gps_y=0.19      ->  
    // lat=49.212688, lon=18.743973, gps_x=23.80, gps_y=-113.99
/*  double p1lo = 18.743650;
    double p1la = 49.213715;
    double p2lo = 18.743973;
    double p2la = 49.212688;
    int dd = 3 * 114;
*/

    // { "N2", 18.743636933, 49.213718281 }  ->  { "M4", 18.744029433, 49.212597581 },
/*  double p1lo = 18.743636933;
    double p1la = 49.213718281;
    double p2lo = 18.744029433;
    double p2la = 49.212597581;
    int dd = 3 * 130;    // distance between p1 and p2 is 130 metres -> 3x to achieve speed 0.33 m/s
*/

    // Park Andreja Hlinku
/*  double p1lo = 17.1618502;  // near S2
    double p1la = 48.1564927;
    double p2lo = 17.1593964;  // S3
    double p2la = 48.1564496;  
    double p3lo = 17.1567450;  // goal
    double p3la = 48.1565383;  */

    //int dd = 3 * 185;    // distance between p1 and p2 is 185 metres -> 3x to achieve speed 0.33 m/s
    //int dd = 10;    // 10 seconds to move to p2
    //int dd2 = 10;   // 10 seconds to move to p3  // real distance is 195m

/*
    // Zamecka zahrada - Lednice, Czech Republic
    double p1lo = 16.8078001;  // S3
    double p1la = 48.7985785;
    double p2lo = 16.804208;   // N4
    double p2la = 48.8025706;  
    double p3lo = 16.8063114;  // M5
    double p3la = 48.7994356;

    int dd = 20;    // 10 seconds to move to p2
    int dd2 = 20;   // 10 seconds to move to p3  // real distance is 195m

    // course = 185;    // course S2 -> S3

    if (getdata_cnt < dd) {
        latitude =  p1la + (p2la - p1la) * getdata_cnt / dd;
        longitude = p1lo + (p2lo - p1lo) * getdata_cnt / dd;
        speed = 0.33;
        //course = 185;    // course S2 -> S3
        course = 330;    // course S3 -> N4
    } else 
    if (getdata_cnt < (dd + dd2)) {
        latitude =  p2la + (p3la - p2la) * (getdata_cnt - dd) / dd2;
        longitude = p2lo + (p3lo - p2lo) * (getdata_cnt - dd) / dd2;
        speed = 0.33;
        //course = 270;    // course S2 -> S3
        course = 156;    // course N4 -> M5
    } else {
        latitude =  p3la;
        longitude = p3lo;
    }
*/
/*
    // Hradza, Bratislava
    latitude =  48.1263699;  // N2
    longitude = 17.1377210;
    speed = 0.3;
    course = 180;
*/
    // FEI STU, Bratislava
//  latitude =  48.1545904;  // N1
//  longitude = 17.0726239;
//  speed = 0.3;
//  course = 180;

/*  latitude =  48.1517470;  // E1
    longitude = 17.0740279;
    speed = 0.3;
    course = 0;
*/
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
