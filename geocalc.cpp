#include "geocalc.h"
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerGeoCalc, "GeoCalc");

#ifdef ISTRO_GEOGRAPHIC_LIB

int GeoCalc::init(void) 
{   
    pgeod = &Geodesic::WGS84();

    return 0;
}

void GeoCalc::close(void)
{
    pgeod = NULL;
} 

void GeoCalc::getDist(double lat1, double lon1, double lat2, double lon2, double& dist, double& azimuth)
{
    double azi2;
    pgeod->Inverse(lat1, lon1, lat2, lon2, dist, azimuth, azi2);
}

#else 

#include <math.h>

const double GEOCALC_ZERO_EPS = 0.000001;

int GeoCalc::init(void) 
{   
    LOGM_WARN(loggerGeoCalc, "init", "MOCK GEOCALC implementation, DEBUG ONLY!!");
    return 0;
}

void GeoCalc::close(void)
{
} 

//double geocalc_dist = 105;

void GeoCalc::getDist(double lat1, double lon1, double lat2, double lon2, double& dist, double& azimuth)
{
    double dx = (lat2 - lat1) / GEOCALC_LATITUDE_1M;
    double dy = (lon2 - lon1) / GEOCALC_LONGITUDE_1M;
    dist = sqrt(dx * dx + dy * dy);
    if ((dx < -GEOCALC_ZERO_EPS) || (dx > GEOCALC_ZERO_EPS) || (dy < -GEOCALC_ZERO_EPS) || (dy > -GEOCALC_ZERO_EPS)) {
        azimuth = atan2(dx, dy) * 180 / M_PI;
        // transform: clockwise from north
        azimuth = 90 - azimuth;
        if (azimuth > 180) {
            azimuth -= 360;
        }
    } else {
        azimuth = 0;
    }
    /*
    dist = geocalc_dist;
    azimuth = 60;
    geocalc_dist -= 20;
    if (geocalc_dist < 0) geocalc_dist = 105;
    */
}

#endif
