#ifndef __NAVIG_H__
#define __NAVIG_H__

#include <stdlib.h>    // NULL
#include "geocalc.h"
#include "navig_data.h"

const int    NAVIGATION_ANGLE_MIN =  30;           // 70;
const int    NAVIGATION_ANGLE_MAX = 150;           // 110; 
const double NAVIGATION_DISTANCE_THRESHOLD  = 12;   /* 6; minimum distance for passing navigation point (metres) */
const double NAVIGATION_DISTANCE_THRESHOLD2 =  3;   /* minimum distance for approaching un/loading area */

const int    NAVIGATION_AREA_NONE = -1;
const int    NAVIGATION_AREA_LOADING = 0;          // navigationPoint[0] is loading area "*1"
const int    NAVIGATION_AREA_UNLOADING = 1;        // navigationPoint[1] is unloading area "*2"
const int    NAVIGATION_AREA_BALLDROP = 10;        // ball drop

int navigation_init(GeoCalc *pGC);

int navigation_point_cnt(void);

int navigation_approach(const char *path, int pos);
int navigation_next_point(const char *path, int &pos, double &point_latitude, double &point_longitude, int &loadarea, int &point_idx);

int navigation_point_get(int point_idx, double &point_latitude, double &point_longitude);
int navigation_point_set(int point_idx, double point_latitude, double point_longitude);

int navigation_ref_set(double ref_latitude, double ref_longitude);
int navigation_ref_get(double& ref_latitude, double& ref_longitude);

int navigation_getXY(double latitude, double longitude, double &x, double &y);
int navigation_getLL(double x, double y, double &latitude, double &longitude);

double navigation_dist2PL(double x, double y, double x1, double y1, double x2, double y2, 
           double *pxx = NULL, double *pyy = NULL); 

#endif
