#ifndef __GEOCALC_H__
#define __GEOCALC_H__

#include "system.h"

#ifdef ISTRO_GEOGRAPHIC_LIB

#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;

#else 

static const double GEOCALC_LONGITUDE_1M = (17.1141433 - 17.1048715) / 690.0;    // { "W6", 17.1048715, 48.1357237 }  ->  { "N6", 17.1141433, 48.1353878 }
static const double GEOCALC_LATITUDE_1M  = (48.1363554 - 48.1333701) / 333.0;    // { "S4", 17.1092313, 48.1333701 }  ->  { "N4", 17.1096112, 48.1363554 }

#endif

class GeoCalc {
private:
#ifdef ISTRO_GEOGRAPHIC_LIB
    const Geodesic *pgeod;
#endif
    
public:
    int init(void);
    void close(void);

    /* dist - distance in metres */
    /* azimuth - is the heading measured clockwise from north (degrees) */
    void getDist(double lat1, double lon1, double lat2, double lon2, double& dist, double& azimuth);
};

#endif
