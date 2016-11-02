#ifndef __GEOCALC_H__
#define __GEOCALC_H__

#include "system.h"

#ifdef ISTRO_GEOGRAPHIC_LIB

#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;

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
    /* azimuth - is the heading measured clockwise from north */
    void getDist(double lat1, double lon1, double lat2, double lon2, double& dist, double& azimuth);
};

#endif
