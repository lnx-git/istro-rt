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

void GeoCalc::getCoord(double lat1, double lon1, double dist, double azimuth, double &lat2, double &lon2)
{
    pgeod->Direct(lat1, lon1, azimuth, dist, lat2, lon2);
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
    double dx = (lon2 - lon1) / GEOCALC_LONGITUDE_1M;
    double dy = (lat2 - lat1) / GEOCALC_LATITUDE_1M;
    dist = sqrt(dx * dx + dy * dy);
    if ((dx < -GEOCALC_ZERO_EPS) || (dx > GEOCALC_ZERO_EPS) || (dy < -GEOCALC_ZERO_EPS) || (dy > -GEOCALC_ZERO_EPS)) {
        // dx and dy are exchanged => to get azimuth clockwise from north
        azimuth = atan2(dx, dy) * 180 / M_PI;
        if (azimuth > 180) {
            azimuth -= 360;
        }
    } else {
        azimuth = 0;
    }
}

void GeoCalc::getCoord(double lat1, double lon1, double dist, double azimuth, double &lat2, double &lon2)
{
    // azimuth is clockwise from north => sin/cos (like navigation_getXY)
    double dx = dist * sin(azimuth * M_PI / 180);
    double dy = dist * cos(azimuth * M_PI / 180);

    lon2 = lon1 + dx * GEOCALC_LONGITUDE_1M;
    lat2 = lat1 + dy * GEOCALC_LATITUDE_1M;
}

#endif
