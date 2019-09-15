#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"    // ANGLE_NONE
#include "navig.h"
#include "logger.h"

LOG_DEFINE(loggerNavig, "navig");

GeoCalc *navigation_pGC = NULL;

double navig_ref_latitude = ANGLE_NONE;
double navig_ref_longitude = ANGLE_NONE;

int navigation_init(GeoCalc *pGC)
{
    navigation_pGC = pGC;

    navig_ref_latitude = ANGLE_NONE;
    navig_ref_longitude = ANGLE_NONE;

    int ref = 0;
    for(int i = 0; i < NAVIGATION_POINT_COUNT; i++) {
        navigationPointXY[i].x = 0;
        navigationPointXY[i].y = 0;
        
        // find first navigation point not starting with '*' -> this will be our reference point
        if (!ref && (navigationPoint[i].name[0] != '*')) {
            ref = navigation_ref_set(navigationPoint[i].latitude, navigationPoint[i].longitude);
        }
    }
    
    // error: reference point was not initialized
    if (!ref) {
        LOGM_ERROR(loggerNavig, "navigation_init", "msg=\"error: reference point not initialized!\"");
        return -1;
    }
    
    return 0;
}

int navigation_point_cnt(void)
{
    return NAVIGATION_POINT_COUNT;
}

int navigation_next_point(const char *path, int &pos, double &point_latitude, double &point_longitude, int &loadarea, int &point_idx)
{
    int ll = strlen(path);

    point_idx = -1;
    point_latitude = ANGLE_NONE;
    point_longitude = ANGLE_NONE;
    loadarea = NAVIGATION_AREA_NONE;
    
    if (pos < ll - 1) {
        for(int idx = 0; idx < NAVIGATION_POINT_COUNT; idx++) {
            char c0 = toupper(path[pos]);
            char c1 = toupper(path[pos+1]);
            // navigacne body v ceste nie su case sensitive
            if ((navigationPoint[idx].name[0] == c0) && (navigationPoint[idx].name[1] == c1)) {
#ifdef ISTRO_NAVIG_BALLDROP
                // v kazdom bode zadanom velkymi pismenami treba vylozit lopticku
                if ((c0 == path[pos]) && (c1 == path[pos+1])) {
                    loadarea = NAVIGATION_AREA_BALLDROP;
                }
#else
                // hviezdickami zacinajuce body *1 a *2 urcuju miesta nakladky a vykladky
                if ((idx == NAVIGATION_AREA_LOADING) || (idx == NAVIGATION_AREA_UNLOADING)) {
                    loadarea = idx;
                }
#endif
                point_latitude = navigationPoint[idx].latitude;
                point_longitude = navigationPoint[idx].longitude;
                point_idx = idx;
                LOGM_INFO(loggerNavig, "navigation_next_point", "msg=\"point found!\", name=\"" 
                    << (char *)navigationPoint[idx].name << "\", pos=" << pos
                    << ", lat=" << ioff(point_latitude, 6) << ", lon=" << ioff(point_longitude, 6)
                    << ", loadarea=" << loadarea << ", idx=" << point_idx);
                pos += 2;
                return 0;
            }
        }
    }

    LOGM_INFO(loggerNavig, "navigation_next_point", "msg=\"point not found!\", pos=" << pos);

    return -1;
}

int navigation_approach(const char *path, int pos)
// vrati 1, ak sa k danemu bodu cesty treba priblizit co najblizsie
{
#ifdef ISTRO_NAVIG_BALLDROP
    char c0 = toupper(path[pos]);
    char c1 = toupper(path[pos+1]);
    return ((c0 == path[pos]) && (c1 == path[pos+1]));  // NAVIGATION_AREA_BALLDROP: ak zadane velkymi pismenami 
#else
    return (path[pos] == '*');    // NAVIGATION_AREA_LOADING or NAVIGATION_AREA_UNLOADING
#endif
}

volatile int navigationPoint_update_pending = 0;

int navigation_point_get(int point_idx, double &point_latitude, double &point_longitude)
{
    point_latitude = ANGLE_NONE;
    point_longitude = ANGLE_NONE;
    
    if ((point_idx < 0) || (point_idx >= NAVIGATION_POINT_COUNT)) {
        return -1;
    }

    if (navigationPoint_update_pending) {
        return -2;
    }
    
    if ((navigationPoint[point_idx].latitude >= ANGLE_OK) || (navigationPoint[point_idx].longitude >= ANGLE_OK)) {
        return 0;
    }
    
    point_latitude = navigationPoint[point_idx].latitude;
    point_longitude = navigationPoint[point_idx].longitude;

    LOGM_INFO(loggerNavig, "navigation_point_get", "msg=\"coordinates acquired!\", name=\"" << (char *)navigationPoint[point_idx].name << "\""
        << ", lat=" << ioff(point_latitude, 6) << ", lon=" << ioff(point_longitude, 6) << ", idx=" << point_idx);
    return 1;
}

int navigation_point_set(int point_idx, double point_latitude, double point_longitude)
{
    if ((point_idx < 0) || (point_idx >= NAVIGATION_POINT_COUNT)) {
        return -1;
    }
    
    navigationPoint_update_pending = 1;
    navigationPoint[point_idx].latitude = point_latitude;
    navigationPoint[point_idx].longitude = point_longitude;
    navigationPoint_update_pending = 0;
    
    navigation_getXY(navigationPoint[point_idx].latitude, navigationPoint[point_idx].longitude, 
        navigationPointXY[point_idx].x, navigationPointXY[point_idx].y);

    LOGM_INFO(loggerNavig, "navigation_point_set", "name=\"" << (char *)navigationPoint[point_idx].name << "\""
        << ", lat=" << ioff(point_latitude, 6) << ", lon=" << ioff(point_longitude, 6) << ", idx=" << point_idx);
    return 0;
}

int navigation_getXY(double latitude, double longitude, double &x, double &y)
{
    double dist, azimuth;

    if ((navig_ref_latitude >= ANGLE_OK) || (navig_ref_longitude >= ANGLE_OK)) {
        x = y = 0;
        return 0;
    }
    
    if ((latitude >= ANGLE_OK) || (longitude >= ANGLE_OK)) {
        x = y = 0;
        return 0;
    }
    
    navigation_pGC->getDist(navig_ref_latitude, navig_ref_longitude, latitude, longitude, dist, azimuth);
    
    x = dist * sin(azimuth * M_PI / 180.0);
    y = dist * cos(azimuth * M_PI / 180.0);
    return 1;
}

int navigation_getLL(double x, double y, double &latitude, double &longitude)
{
    if ((navig_ref_latitude >= ANGLE_OK) || (navig_ref_longitude >= ANGLE_OK)) {
        latitude = longitude = ANGLE_NONE;
        return 0;
    }

    double azimuth = atan2(x, y) * 180 / M_PI;
    double dist = sqrt(x * x + y * y);
    
    navigation_pGC->getCoord(navig_ref_latitude, navig_ref_longitude, dist, azimuth, latitude, longitude);
    
    return 1;
}

void navigation_points_calcXY(void)
{
    double navp_xmin = 0;
    double navp_xmax = 0;
    double navp_ymin = 0;
    double navp_ymax = 0;

    for(int i = 0; i < NAVIGATION_POINT_COUNT; i++) {
        if (navigation_getXY(navigationPoint[i].latitude, navigationPoint[i].longitude, navigationPointXY[i].x, navigationPointXY[i].y)) {
            if (i == 0) {
                navp_xmin = navigationPointXY[i].x;
                navp_xmax = navigationPointXY[i].x;
                navp_ymin = navigationPointXY[i].y;
                navp_ymax = navigationPointXY[i].y;
            } else {
                if (navp_xmin > navigationPointXY[i].x) navp_xmin = navigationPointXY[i].x;
                if (navp_xmax < navigationPointXY[i].x) navp_xmax = navigationPointXY[i].x;
                if (navp_ymin > navigationPointXY[i].y) navp_ymin = navigationPointXY[i].y;
                if (navp_ymax < navigationPointXY[i].y) navp_ymax = navigationPointXY[i].y;
            }
        }
    }

    LOGM_INFO(loggerNavig, "navigation_points_calcXY", "ref_latitude=" << ioff(navig_ref_latitude, 6) << ", ref_longitude=" << ioff(navig_ref_longitude, 6)
        << ", navp_xmin=" << ioff(navp_xmin, 0) << ", navp_xmax=" << ioff(navp_xmax, 0) << ", navp_ymin=" << ioff(navp_ymin, 0) << ", navp_ymax=" << ioff(navp_ymax, 0));
}

int navigation_ref_set(double ref_latitude, double ref_longitude)
{  
    navig_ref_latitude = ref_latitude;
    navig_ref_longitude = ref_longitude;
    
    int ref = (navig_ref_latitude < ANGLE_OK) && (navig_ref_longitude < ANGLE_OK);

    LOGM_INFO(loggerNavig, "navigation_ref_set", "ref=" << ref << ", ref_latitude=" << ioff(navig_ref_latitude, 6) << ", ref_longitude=" << ioff(navig_ref_longitude, 6));
    
    navigation_points_calcXY();
    
    return ref;
}

int navigation_ref_get(double& ref_latitude, double& ref_longitude)
{
    ref_latitude = navig_ref_latitude;
    ref_longitude = navig_ref_longitude;
    
    return (navig_ref_latitude < ANGLE_OK) && (navig_ref_longitude < ANGLE_OK);
}

const double NAVIG_ZERO_EPS = 0.000001;

double navigation_dist2PL(double x, double y, double x1, double y1, double x2, double y2, double *pxx, double *pyy) 
// squared distance between point (x, y) and line segment (x1, y1) -> (x2, y2)
{
    /* segment direction vector */
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    /* squared segment length */
    double l2 = dx * dx + dy * dy;
    
    /* dot product: vectors from (x1, y1) */
    double dd = (x - x1) * dx + (y - y1) * dy; 
    
    /* calculate  - avoid division by zero */
    double p = 0;
    if (l2 > NAVIG_ZERO_EPS) {
        p = dd / l2;
    }
    
    /* calculate projection point */
    double xx = x1;
    double yy = y1;
    
    if (p > 1) {
        xx = x2;
        yy = y2;
    } else
    if (p > 0) {
        xx += p * dx;
        yy += p * dy;
    }

    if ((pxx != NULL) && (pyy != NULL)) {
        *pxx = xx;  
        *pyy = yy;
    }

    double dxx = x - xx;
    double dyy = y - yy;
    
    return dxx * dxx + dyy * dyy;
}
