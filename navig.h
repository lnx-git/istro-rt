#ifndef __NAVIG_H__
#define __NAVIG_H__

const int    NAVIGATION_ANGLE_MIN =  30;           // 70;
const int    NAVIGATION_ANGLE_MAX = 150;           // 110; 
const double NAVIGATION_DISTANCE_THRESHOLD  = 12;   /* minimum distance for passing navigation point (metres) */
const double NAVIGATION_DISTANCE_THRESHOLD2 =  3;   /* minimum distance for approaching un/loading area */

const int    NAVIGATION_AREA_NONE = -1;
const int    NAVIGATION_AREA_LOADING = 0;          // navigationPoint[0] is loading area "*1"
const int    NAVIGATION_AREA_UNLOADING = 1;        // navigationPoint[1] is unloading area "*2"

typedef struct { 
    char  name[3];
    double longitude;
    double latitude;
} nav_point_t;

extern nav_point_t navigationPoint[];

int navigation_approach(const char *path, int pos);
int navigation_next_point(const char *path, int &pos, double &point_latitude, double &point_longitude, int &loadarea);

#endif
