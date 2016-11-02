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

int GeoCalc::init(void) 
{   
    LOGM_WARN(loggerGeoCalc, "init", "MOCK GEOCALC implementation, DEBUG ONLY!!");
    return 0;
}

void GeoCalc::close(void)
{
} 

double geocalc_dist = 105;

void GeoCalc::getDist(double lat1, double lon1, double lat2, double lon2, double& dist, double& azimuth)
{
    dist = geocalc_dist;
    azimuth = 60;
    geocalc_dist -= 20;
    if (geocalc_dist < 0) geocalc_dist = 105;
}

#endif
