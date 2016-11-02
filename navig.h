#ifndef __NAVIG_H__
#define __NAVIG_H__

const int    NAVIGATION_ANGLE_MIN = 70;
const int    NAVIGATION_ANGLE_MAX = 110;
const double NAVIGATION_DISTANCE_THRESHOLD = 10;   /* minimum distance for passing navigation point (metres)*/

typedef struct { 
    char  name[3];
    double longitude;
    double latitude;
} nav_point_t;

int navigation_next_point(const char *path, int &pos, double &point_latitude, double &point_longitude);

#endif
