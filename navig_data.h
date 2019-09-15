#ifndef __NAVIG_DATA_H__
#define __NAVIG_DATA_H__

extern const int NAVIGATION_POINT_COUNT;

typedef struct { 
    char  name[3];        // fixme: kedze pole je volatile, tak pri praci so stringom treba explicitne vzdy pretypovat na (char *)
    double longitude;
    double latitude;
} nav_point_t;

extern volatile nav_point_t navigationPoint[];

typedef struct { 
    double x;
    double y;
} nav_point_xy_t;

extern nav_point_xy_t navigationPointXY[];

#endif
